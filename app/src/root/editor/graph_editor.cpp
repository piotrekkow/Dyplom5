#include "graph_editor.hpp"

#include <expected>
#include <road_network/graph.hpp>
#include <road_network/util/compute_helpers.hpp>

#include "bus/dirty_event_bus.hpp"
#include "bus/network_event_bus.hpp"
#include "regulations/types/stream_kind.hpp"
#include "util/apply_tx.hpp"

// creation
NodeId GraphEditor::createNode(Transaction& tx, Position center) {
    auto mut = graph_.createNode(center);
    const auto nid = mut.id;
    return applyTx(tx, std::move(mut), bus_.nodeCreated, Created<NodeId>{nid},
                   bus_.nodeRemoved, Removed<NodeId>{nid});
}
EntryId GraphEditor::createEntry(Transaction& tx, NodeId at, Position position,
                                 Vector2 heading) {
    auto mut = graph_.createEntry(at, position, heading.normalized());
    const auto eid = mut.id;
    return applyTx(tx, std::move(mut), bus_.entryCreated,
                   Created<EntryId, NodeId>{.id = eid, .at = at},
                   bus_.entryRemoved,
                   Removed<EntryId, NodeId>{.id = eid, .at = at});
}
ExitId GraphEditor::createExit(Transaction& tx, NodeId at, Position position,
                               Vector2 heading) {
    auto mut = graph_.createExit(at, position, heading.normalized());
    const auto xid = mut.id;
    return applyTx(tx, std::move(mut), bus_.exitCreated,
                   Created<ExitId, NodeId>{.id = xid, .at = at},
                   bus_.exitRemoved,
                   Removed<ExitId, NodeId>{.id = xid, .at = at});
}
CrosswalkSeriesId GraphEditor::createCrosswalkSeries(Transaction& tx,
                                                     NodeId at) {
    auto mut = graph_.createCrosswalkSeries(at);
    const auto csid = mut.id;
    return applyTx(tx, std::move(mut), bus_.crosswalkSeriesCreated,
                   Created<CrosswalkSeriesId, NodeId>{.id = csid, .at = at},
                   bus_.crosswalkSeriesRemoved,
                   Removed<CrosswalkSeriesId, NodeId>{.id = csid, .at = at});
}
CrosswalkId GraphEditor::createCrosswalk(Transaction& tx,
                                         CrosswalkSeriesId parent,
                                         Line anchor) {
    auto mut = graph_.createCrosswalk(parent, anchor);
    const auto at = mut.at;
    const auto cid = mut.id;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    return applyTx(tx, std::move(mut), bus_.crosswalkCreated,
                   Created<CrosswalkId, NodeId>{.id = cid, .at = at},
                   bus_.crosswalkRemoved,
                   Removed<CrosswalkId, NodeId>{.id = cid, .at = at});
}
std::expected<std::vector<ClusterId>, MovementLayoutError>
GraphEditor::setEntryLayout(Transaction& tx, EntryId id,
                            std::vector<MovementSpec> specs,
                            std::vector<bool> shared) {
    if (auto v = validateLayout(id, specs, shared); !v)
        return std::unexpected(v.error());

    clearEntryLayout(tx, id);

    std::vector<MovementId> backlog;
    std::vector<ClusterId> clusters;
    int laneCursor = 0;
    int clFirst = 0;
    for (size_t i = 0; i < specs.size(); ++i) {
        auto mId = createMovement(tx, id, specs[i].exit, laneCursor,
                                  specs[i].laneCount);
        backlog.push_back(mId);

        laneCursor += specs[i].laneCount;
        if (i < specs.size() - 1 && shared[i])
            --laneCursor;
        else {
            auto cId =
                createCluster(tx, id, backlog, clFirst, laneCursor - clFirst);
            clusters.push_back(cId);
            backlog.clear();
            clFirst = laneCursor;
        }
    }
    const auto at = graph_.parentOf(id);
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    return clusters;
}

std::expected<void, MovementLayoutError> GraphEditor::validateLayout(
    EntryId id, const std::vector<GraphEditor::MovementSpec>& specs,
    const std::vector<bool>& shared) {
    const auto* e = graph_.data().entries().get(id);

    auto specLanes = [&] {
        int count = 0;
        for (auto& s : specs) count += s.laneCount;
        for (auto s : shared)
            if (s) --count;
        return count;
    };

    assert(!specs.empty());

    if (specs.size() != shared.size() + 1)
        return std::unexpected(MovementLayoutError::SpecSizeMismatch);

    if (e->params().lanes.empty())
        return std::unexpected(MovementLayoutError::EmptyLaneSet);

    if (specLanes() != static_cast<int>(e->params().lanes.size()))
        return std::unexpected(MovementLayoutError::LaneCountMismatch);

    if (!std::ranges::all_of(specs, [&](auto& s) { return s.laneCount > 0; }))
        assert(false && "Movement layout : Movement with zero lanes");

    if (!std::ranges::all_of(specs, [&](auto& s) {
            const auto* x = graph_.data().exits().get(s.exit);
            return x && e->at() == x->at();
        }))
        assert(false && "Movement layout : Exit from spec not in scope");

    // Movements are born with a 1:1 entry→exit lane split, so a movement can't
    // claim more entry lanes than its exit has lanes without the geometry solve
    // indexing a nonexistent exit lane.
    if (!std::ranges::all_of(specs, [&](auto& s) {
            const auto* x = graph_.data().exits().get(s.exit);
            return x != nullptr &&
                   s.laneCount <= static_cast<int>(x->params().lanes.size());
        }))
        return std::unexpected(MovementLayoutError::ExitLaneOverflow);

    auto angles = specs | std::views::transform([&](auto& s) {
                      return compute::classifyAngle(graph_.data(), id, s.exit);
                  });
    if (!std::ranges::is_sorted(angles, std::ranges::greater{}))
        return std::unexpected(MovementLayoutError::AngularOrderViolation);

    return {};
}

// removal
void GraphEditor::removeNode(Transaction& tx, NodeId id) {
    const auto* n = graph_.data().nodes().get(id);
    auto csCopy = n->crosswalkSeries();
    auto eCopy = n->entries();
    auto xCopy = n->exits();
    for (auto csid : csCopy) removeCrosswalkSeries(tx, csid);
    for (auto eid : eCopy) removeEntry(tx, eid);
    for (auto xid : xCopy) removeExit(tx, xid);

    auto mut = graph_.removeNode(id);
    dirty_.nodeRemoved.publish(RemovedTx<NodeId>{.id = id, .tx = tx});
    applyTx(tx, std::move(mut), bus_.nodeRemoved, Removed<NodeId>{id},
            bus_.nodeCreated, Created<NodeId>{id});
}
void GraphEditor::removeEntry(Transaction& tx, EntryId id) {
    clearEntryLayout(tx, id);

    auto mut = graph_.removeEntry(id);
    const auto at = mut.at;
    dirty_.entryRemoved.publish(
        RemovedTx<EntryId, NodeId>{.id = id, .at = at, .tx = tx});
    applyTx(tx, std::move(mut), bus_.entryRemoved,
            Removed<EntryId, NodeId>{.id = id, .at = at}, bus_.entryCreated,
            Created<EntryId, NodeId>{.id = id, .at = at});
}
void GraphEditor::removeExit(Transaction& tx, ExitId id) {
    for (auto eid : entriesFeedingExit(id)) clearEntryLayout(tx, eid);

    auto mut = graph_.removeExit(id);
    const auto at = mut.at;
    applyTx(tx, std::move(mut), bus_.exitRemoved,
            Removed<ExitId, NodeId>{.id = id, .at = at}, bus_.exitCreated,
            Created<ExitId, NodeId>{.id = id, .at = at});
}
void GraphEditor::removeCrosswalk(Transaction& tx, CrosswalkId id) {
    auto mut = graph_.removeCrosswalk(id);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    dirty_.crosswalkRemoved.publish(
        RemovedTx<CrosswalkId, NodeId>{.id = id, .at = at, .tx = tx});
    applyTx(tx, std::move(mut), bus_.crosswalkRemoved,
            Removed<CrosswalkId, NodeId>{.id = id, .at = at},
            bus_.crosswalkCreated,
            Created<CrosswalkId, NodeId>{.id = id, .at = at});
}
void GraphEditor::removeCrosswalkSeries(Transaction& tx, CrosswalkSeriesId id) {
    const auto* cs = graph_.data().crosswalkSeries().get(id);
    auto cwCopy = cs->crosswalks();
    for (auto cid : cwCopy) removeCrosswalk(tx, cid);

    auto mut = graph_.removeCrosswalkSeries(id);
    const auto at = mut.at;
    applyTx(tx, std::move(mut), bus_.crosswalkSeriesRemoved,
            Removed<CrosswalkSeriesId, NodeId>{.id = id, .at = at},
            bus_.crosswalkSeriesCreated,
            Created<CrosswalkSeriesId, NodeId>{.id = id, .at = at});
}
void GraphEditor::clearEntryLayout(Transaction& tx, EntryId id) {
    const auto* e = graph_.data().entries().get(id);
    auto clCopy = e->clusters();
    auto mCopy = e->movements();
    for (auto cid : clCopy) removeCluster(tx, cid);
    for (auto mid : mCopy) removeMovement(tx, mid);
    const auto node = e->at();
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
}

// override handling
void GraphEditor::setPermittedOverride(Transaction& tx, NodeId node, StreamId a,
                                       StreamId b, bool permitted) {
    auto mut = graph_.setPermittedOverride(node, a, b, permitted);
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.streamPermittedOverrideSet,
                  Set<StreamPermittedOverrideTag, NodeId>{.at = node});
}
void GraphEditor::clearPermittedOverride(Transaction& tx, NodeId node,
                                         StreamId a, StreamId b) {
    auto mut = graph_.clearPermittedOverride(node, a, b);
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.streamPermittedOverrideCleared,
                  Cleared<StreamPermittedOverrideTag, NodeId>{.at = node});
}
void GraphEditor::setIntergreenOverride(Transaction& tx, NodeId node,
                                        StreamId evac, StreamId appr,
                                        int t_maxEvAp, int t_maxApEv) {
    auto mut =
        graph_.setIntergreenOverride(node, evac, appr, t_maxEvAp, t_maxApEv);
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.streamIntergreenOverrideSet,
                  Set<StreamIntergreenOverrideTag, NodeId>{.at = node});
}
void GraphEditor::clearIntergreenOverride(Transaction& tx, NodeId node,
                                          StreamId evac, StreamId appr) {
    auto mut = graph_.clearIntergreenOverride(node, evac, appr);
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.streamIntergreenOverrideCleared,
                  Cleared<StreamIntergreenOverrideTag, NodeId>{.at = node});
}

// setters
void GraphEditor::setEntryLanes(Transaction& tx, EntryId id,
                                std::vector<EntryLaneSpec> spec) {
    auto* e = graph_.data().entries().get(id);
    if (spec.size() != e->params().lanes.size()) clearEntryLayout(tx, id);

    auto params = e->params();
    params.lanes = std::move(spec);

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.entryUpdated,
                  Updated<EntryId, NodeId>{.id = id, .at = at});
}
void GraphEditor::setExitLanes(Transaction& tx, ExitId id,
                               std::vector<double> spec) {
    auto* x = graph_.data().exits().get(id);
    if (spec.size() != x->params().lanes.size())
        for (auto eid : entriesFeedingExit(id)) clearEntryLayout(tx, eid);

    auto params = x->params();
    params.lanes = std::move(spec);

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.exitUpdated,
                  Updated<ExitId, NodeId>{.id = id, .at = at});
}

void GraphEditor::moveEntry(Transaction& tx, EntryId id, Position position,
                            Vector2 heading) {
    auto* e = graph_.data().entries().get(id);

    auto params = e->params();
    params.position = position;
    params.heading = heading;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.entryUpdated,
                  Updated<EntryId, NodeId>{.id = id, .at = at});
}
void GraphEditor::moveExit(Transaction& tx, ExitId id, Position position,
                           Vector2 heading) {
    auto* x = graph_.data().exits().get(id);

    auto params = x->params();
    params.position = position;
    params.heading = heading;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.exitUpdated,
                  Updated<ExitId, NodeId>{.id = id, .at = at});
}

void GraphEditor::setEntrySpeedLimit(Transaction& tx, EntryId id,
                                     double speedLimit) {
    auto* e = graph_.data().entries().get(id);
    assert(speedLimit > 0 && "speed limit has to be positive");

    auto params = e->params();
    params.speedLimit = speedLimit;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.entryUpdated,
                  Updated<EntryId, NodeId>{.id = id, .at = at});
}
void GraphEditor::setExitSpeedLimit(Transaction& tx, ExitId id,
                                    double speedLimit) {
    auto* x = graph_.data().exits().get(id);
    assert(speedLimit > 0 && "speed limit has to be positive");

    auto params = x->params();
    params.speedLimit = speedLimit;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.exitUpdated,
                  Updated<ExitId, NodeId>{.id = id, .at = at});
}

void GraphEditor::setMovementPathSpec(Transaction& tx, MovementId id,
                                      MovementPathSpec pathSpec) {
    auto* m = graph_.data().movements().get(id);
    assert(pathSpec.dE > -1e-9 && pathSpec.dX > -1e-9 &&
           "offsets have to be positive");

    auto params = m->params();
    params.pathSpec = pathSpec;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.movementUpdated,
                  Updated<MovementId, NodeId>{.id = id, .at = at});
}
void GraphEditor::setMovementType(Transaction& tx, MovementId id,
                                  MovementType type) {
    auto* m = graph_.data().movements().get(id);

    auto params = m->params();
    params.type = type;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.movementUpdated,
                  Updated<MovementId, NodeId>{.id = id, .at = at});
}
void GraphEditor::setMovementSplitPerLane(Transaction& tx, MovementId id,
                                          std::vector<int> splitPerLane) {
    auto* m = graph_.data().movements().get(id);
    assert(m->lanes().laneCount == splitPerLane.size());

    auto xCount = graph_.data().exits().get(m->to())->params().lanes.size();
    int splitCount = 0;
    for (int split : splitPerLane) splitCount += split;
    assert(xCount == splitCount);

    auto params = m->params();
    params.splitPerLane = std::move(splitPerLane);

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.movementUpdated,
                  Updated<MovementId, NodeId>{.id = id, .at = at});
}

void GraphEditor::setCSeriesCriticalPath(Transaction& tx, CrosswalkSeriesId id,
                                         Polyline path) {
    auto* cs = graph_.data().crosswalkSeries().get(id);

    auto params = cs->params();
    params.criticalGreenPath = std::move(path);

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.crosswalkSeriesUpdated,
                  Updated<CrosswalkSeriesId, NodeId>{.id = id, .at = at});
}
void GraphEditor::setCSeriesCriticalSpeed(Transaction& tx, CrosswalkSeriesId id,
                                          double speed) {
    auto* cs = graph_.data().crosswalkSeries().get(id);
    assert(speed > 0 && "walk speed must be positive");

    auto params = cs->params();
    params.criticalGreenSpeedOverride = speed;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.crosswalkSeriesUpdated,
                  Updated<CrosswalkSeriesId, NodeId>{.id = id, .at = at});
}

void GraphEditor::setCSeriesType(Transaction& tx, CrosswalkSeriesId id,
                                 CrosswalkType type) {
    auto* cs = graph_.data().crosswalkSeries().get(id);

    auto params = cs->params();
    params.type = type;

    auto resizeCrosswalkToMatchType = [&](CrosswalkId id) {
        bool needsResize =
            (cs->params().type == CrosswalkType::PEDESTRIAN_AND_CYCLIST ||
             type == CrosswalkType::PEDESTRIAN_AND_CYCLIST) &&
            (cs->params().type != type);
        if (!needsResize) return;
        auto cinput = graph_.data().crosswalks().get(id)->params().input;
        if (type == CrosswalkType::PEDESTRIAN_AND_CYCLIST)
            cinput.split.width /= 2.0;
        else
            cinput.split.width = cinput.edge.width;

        setCrosswalkGeometry(tx, id, cinput);
    };

    for (auto cid : cs->crosswalks()) resizeCrosswalkToMatchType(cid);

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.crosswalkSeriesUpdated,
                  Updated<CrosswalkSeriesId, NodeId>{.id = id, .at = at});
}

void GraphEditor::setCrosswalkGeometry(Transaction& tx, CrosswalkId id,
                                       CrosswalkGeometryInput geometry) {
    auto* cw = graph_.data().crosswalks().get(id);
    auto cType =
        graph_.data().crosswalkSeries().get(cw->parent())->params().type;
    assert(cType == CrosswalkType::PEDESTRIAN_AND_CYCLIST
               ? geometry.split.width != geometry.edge.width
               : geometry.split.width == geometry.edge.width);

    auto params = cw->params();
    params.input = geometry;

    auto mut = graph_.setParams(id, params);
    const auto at = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = at, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.crosswalkUpdated,
                  Updated<CrosswalkId, NodeId>{.id = id, .at = at});
}

void GraphEditor::registerSolve(Transaction& tx) {
    reNodes_.clear();
    tx.addSolve(SolvePriority::GRAPH, {.rederive = [this] { rederive(); },
                                       .flush = [this] { flush(); }});
}

void GraphEditor::rederive() {
    auto r = graph_.solve();
    reNodes_.insert(r.nodes.begin(), r.nodes.end());
}

void GraphEditor::flush() {
    for (auto id : reNodes_)
        if (graph_.data().nodes().alive(id))
            bus_.nodeUpdated.publish(Updated<NodeId>{id});

    reNodes_.clear();
}

// private
MovementId GraphEditor::createMovement(Transaction& tx, EntryId from, ExitId to,
                                       int firstIdx, int laneCount) {
    auto mut = graph_.createMovement(
        from, to, {.at = from, .firstIdx = firstIdx, .laneCount = laneCount});
    const auto at = mut.at;
    const auto id = mut.id;

    applyTx(tx, std::move(mut), bus_.movementCreated,
            Created<MovementId, NodeId>{.id = id, .at = at},
            bus_.movementRemoved,
            Removed<MovementId, NodeId>{.id = id, .at = at});
    dirty_.movementCreated.publish(
        CreatedTx<MovementId, NodeId>{.id = id, .at = at, .tx = tx});
    return id;
}
ClusterId GraphEditor::createCluster(Transaction& tx, EntryId at,
                                     std::vector<MovementId> movements,
                                     int firstIdx, int laneCount) {
    auto mut = graph_.createCluster(
        at, std::move(movements),
        {.at = at, .firstIdx = firstIdx, .laneCount = laneCount});
    const auto nid = mut.at;
    const auto cid = mut.id;
    return applyTx(tx, std::move(mut), bus_.clusterCreated,
                   Created<ClusterId, NodeId>{.id = cid, .at = nid},
                   bus_.clusterRemoved,
                   Removed<ClusterId, NodeId>{.id = cid, .at = nid});
}

void GraphEditor::removeMovement(Transaction& tx, MovementId id) {
    auto mut = graph_.removeMovement(id);
    const auto at = mut.at;
    dirty_.movementRemoved.publish(
        RemovedTx<MovementId, NodeId>{.id = id, .at = at, .tx = tx});
    applyTx(tx, std::move(mut), bus_.movementRemoved,
            Removed<MovementId, NodeId>{.id = id, .at = at},
            bus_.movementCreated,
            Created<MovementId, NodeId>{.id = id, .at = at});
}

void GraphEditor::removeCluster(Transaction& tx, ClusterId id) {
    auto mut = graph_.removeCluster(id);
    const auto at = mut.at;
    applyTx(tx, std::move(mut), bus_.clusterRemoved,
            Removed<ClusterId, NodeId>{.id = id, .at = at}, bus_.clusterCreated,
            Created<ClusterId, NodeId>{.id = id, .at = at});
}

std::vector<EntryId> GraphEditor::entriesFeedingExit(ExitId exit) {
    const auto* x = graph_.data().exits().get(exit);
    const auto* n = graph_.data().nodes().get(x->at());
    std::vector<EntryId> result;
    for (auto eid : n->entries()) {
        const auto* e = graph_.data().entries().get(eid);
        bool feeds = std::ranges::any_of(e->movements(), [&](MovementId mid) {
            return graph_.data().movements().get(mid)->to() == exit;
        });
        if (feeds) result.push_back(eid);
    }
    return result;
}