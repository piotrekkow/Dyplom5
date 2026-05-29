#include "graph_editor.hpp"

#include <algorithm>
#include <expected>
#include <functional>
#include <ranges>
#include <road_network/graph.hpp>
#include <road_network/graph/derived/graph_geometry.hpp>
#include <road_network/util/geometry/line.hpp>

#include "bus/network_event_bus.hpp"
#include "bus/network_events.hpp"
#include "road_network/util/ids.hpp"
#include "root/command/solve_priority.hpp"
#include "root/command/transaction.hpp"
#include "root/editor/errors/graph_errors.hpp"

//
//  CREATE TX
//

NodeId GraphEditor::createNodeTx(Transaction& tx, Position center) {
    auto mut = graph_.createNode(center);
    tx.recordUndo(std::move(mut.undo), bus_.nodeRemoved, NodeRemoved{mut.id});
    tx.recordEvent(bus_.nodeCreated, NodeCreated{mut.id});
    onNodeStale(tx, mut.id);
    return mut.id;
}

EntryId GraphEditor::createEntryTx(Transaction& tx, NodeId at,
                                   Position position, Vector2 heading) {
    auto mut = graph_.createEntry(at, position, heading);
    tx.recordUndo(std::move(mut.undo), bus_.entryRemoved,
                  EntryRemoved{mut.id, at});
    tx.recordEvent(bus_.entryCreated, EntryCreated{mut.id, at});
    onNodeStale(tx, at);
    return mut.id;
}

ExitId GraphEditor::createExitTx(Transaction& tx, NodeId at, Position position,
                                 Vector2 heading) {
    auto mut = graph_.createExit(at, position, heading);
    tx.recordUndo(std::move(mut.undo), bus_.exitRemoved,
                  ExitRemoved{mut.id, at});
    tx.recordEvent(bus_.exitCreated, ExitCreated{mut.id, at});
    onNodeStale(tx, at);
    return mut.id;
}

CrosswalkSeriesId GraphEditor::createCrosswalkSeriesTx(Transaction& tx,
                                                       NodeId at) {
    auto mut = graph_.createCrosswalkSeries(at);
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkSeriesRemoved,
                  CrosswalkSeriesRemoved{mut.id, at});
    tx.recordEvent(bus_.crosswalkSeriesCreated,
                   CrosswalkSeriesCreated{mut.id, at});
    onNodeStale(tx, at);
    return mut.id;
}

CrosswalkId GraphEditor::createCrosswalkTx(Transaction& tx,
                                           CrosswalkSeriesId parent,
                                           Line line) {
    auto mut = graph_.createCrosswalk(parent, line);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkRemoved,
                  CrosswalkRemoved{mut.id, nodeId});
    tx.recordEvent(bus_.crosswalkCreated, CrosswalkCreated{mut.id, nodeId});
    onNodeStale(tx, nodeId);
    return mut.id;
}

void GraphEditor::setEntryLanesTx(Transaction& tx, EntryId at,
                                  std::vector<EntryLaneSpec> specs) {
    auto mut = graph_.setEntryLanes(at, std::move(specs));
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.entryLanesChanged,
                  EntryLanesChanged{at, nodeId});
    tx.recordEvent(bus_.entryLanesChanged, EntryLanesChanged{at, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setExitLanesTx(Transaction& tx, ExitId at,
                                 std::vector<ExitLaneSpec> specs) {
    auto mut = graph_.setExitLanes(at, std::move(specs));
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.exitLanesChanged,
                  ExitLanesChanged{at, nodeId});
    tx.recordEvent(bus_.exitLanesChanged, ExitLanesChanged{at, nodeId});
    onNodeStale(tx, nodeId);
}

//
//  SETTERS TX
//

std::expected<void, MovementLayoutError> GraphEditor::validateLayout(
    EntryId id, const std::vector<MovementSpec>& specs,
    const std::vector<bool>& shared) {
    const auto* e = graph_.data().entries().get(id);

    if (!specs.empty()) {
        if (specs.size() != shared.size() + 1)
            return std::unexpected(MovementLayoutError::SpecSizeMismatch);
        if (e->lanes().empty())
            return std::unexpected(MovementLayoutError::EmptyLaneSet);
        int specLanes = 0;
        for (auto& s : specs) specLanes += s.laneCount;
        for (auto s : shared)
            if (s) --specLanes;
        if (specLanes != static_cast<int>(e->lanes().size()))
            return std::unexpected(MovementLayoutError::LaneCountMismatch);

        if (!std::ranges::all_of(specs,
                                 [&](auto& s) { return s.laneCount > 0; }))
            assert(false &&
                   "Movement layout validation : Movement with zero lanes");

        if (!std::ranges::all_of(specs, [&](auto& s) {
                const auto* x = graph_.data().exits().get(s.exit);
                return x && e->at() == x->at();
            }))
            assert(false &&
                   "Movement layout validation : Exit from spec not in scope");

        auto angles =
            specs | std::views::transform([&](auto& s) {
                return graph_geometry::classifyAngle(graph_.data(), id, s.exit);
            });
        if (!std::ranges::is_sorted(angles, std::ranges::greater{}))
            return std::unexpected(MovementLayoutError::AngularOrderViolation);
    }
    return {};
}

std::expected<void, MovementSplitError> GraphEditor::validateSplits(
    MovementId id, const std::vector<int>& splits) {
    auto m = graph_.data().movements().get(id);
    auto x = graph_.data().exits().get(m->to());
    if (m->lanes().size() != splits.size())
        return std::unexpected(MovementSplitError::LaneCountToSplitMismatch);
    int xSizeExpected = 0;
    for (auto val : splits) {
        assert(val > 0 && "Movement layout validation : Zero split");
        xSizeExpected += val;
    }
    if (static_cast<int>(x->lanes().size()) != xSizeExpected)
        return std::unexpected(MovementSplitError::ExitLaneCountSplitMismatch);
    return {};
}

// Validation and coordination with other domains performed by EditorContext
std::vector<MovementId> GraphEditor::setEntryLayoutTx(
    Transaction& tx, EntryId id, std::vector<GraphEditor::MovementSpec> specs,
    std::vector<bool> shared) {
    clearEntryLayoutTx(tx, id);

    const auto* e = graph_.data().entries().get(id);

    std::vector<MovementId> movementBacklog;
    std::vector<MovementId> allMovements;
    std::vector<EntryLaneId> laneBacklog;
    size_t laneCursor = 0;
    for (size_t i = 0; i < specs.size(); ++i) {
        std::vector<EntryLaneId> movementLanes;
        size_t specEnd = laneCursor + static_cast<size_t>(specs[i].laneCount);
        while (laneCursor < specEnd) {
            EntryLaneId next = e->lanes()[laneCursor];
            movementLanes.push_back(next);
            if (laneBacklog.empty() || laneBacklog.back() != next)
                laneBacklog.push_back(next);
            ++laneCursor;
        }

        auto defaultSplit = [&](MovementSpec& ms) -> std::vector<int> {
            const auto* x = graph_.data().exits().get(ms.exit);
            std::vector<int> result(ms.laneCount, 1);
            result.back() = x->lanes().size() - ms.laneCount + 1;
            return result;
        };

        MovementId newMovement =
            createMovementTx(tx, id, specs[i].exit, std::move(movementLanes),
                             defaultSplit(specs[i]));
        movementBacklog.push_back(newMovement);
        allMovements.push_back(newMovement);

        if (i < specs.size() - 1 && shared[i]) {
            --laneCursor;
        } else {
            createClusterTx(tx, id, movementBacklog, laneBacklog);
            movementBacklog.clear();
            laneBacklog.clear();
        }
    }
    return allMovements;
}

std::expected<void, MovementSplitError> GraphEditor::setMovementSplitsTx(
    Transaction& tx, MovementId id, std::vector<int> splits) {
    auto err = validateSplits(id, splits);
    if (!err) return std::unexpected(err.error());
    auto mut = graph_.setMovementSplits(id, std::move(splits));
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.movementUpdated,
                  MovementUpdated{id, nodeId});
    tx.recordEvent(bus_.movementUpdated, MovementUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
    return {};
}

void GraphEditor::setMovementPathSpecTx(Transaction& tx, MovementId id,
                                        MovementPathSpec spec) {
    auto mut = graph_.setMovementPathSpec(id, spec);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.movementUpdated,
                  MovementUpdated{id, nodeId});
    tx.recordEvent(bus_.movementUpdated, MovementUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setMovementTypeTx(Transaction& tx, MovementId id,
                                    MovementType type) {
    auto mut = graph_.setMovementType(id, type);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.movementUpdated,
                  MovementUpdated{id, nodeId});
    tx.recordEvent(bus_.movementUpdated, MovementUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setCrosswalkSeriesPedPathTx(Transaction& tx,
                                              CrosswalkSeriesId id,
                                              Polyline path) {
    auto mut = graph_.setCrosswalkSeriesPedestrianPath(id, std::move(path));
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkSeriesUpdated,
                  CrosswalkSeriesUpdated{id, nodeId});
    tx.recordEvent(bus_.crosswalkSeriesUpdated,
                   CrosswalkSeriesUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setCrosswalkTypeTx(Transaction& tx, CrosswalkId id,
                                     CrosswalkType type) {
    auto mut = graph_.setCrosswalkType(id, type);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkUpdated,
                  CrosswalkUpdated{id, nodeId});
    tx.recordEvent(bus_.crosswalkUpdated, CrosswalkUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setCrosswalkLineTx(Transaction& tx, CrosswalkId id,
                                     Line line) {
    auto mut = graph_.setCrosswalkLine(id, line);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkUpdated,
                  CrosswalkUpdated{id, nodeId});
    tx.recordEvent(bus_.crosswalkUpdated, CrosswalkUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setCrosswalkEdgeTx(Transaction& tx, CrosswalkId id,
                                     Crosswalk::SegmentParams edge) {
    auto mut = graph_.setCrosswalkEdge(id, edge);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkUpdated,
                  CrosswalkUpdated{id, nodeId});
    tx.recordEvent(bus_.crosswalkUpdated, CrosswalkUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setCrosswalkSplitTx(Transaction& tx, CrosswalkId id,
                                      Crosswalk::SegmentParams split) {
    auto mut = graph_.setCrosswalkSplit(id, split);
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkUpdated,
                  CrosswalkUpdated{id, nodeId});
    tx.recordEvent(bus_.crosswalkUpdated, CrosswalkUpdated{id, nodeId});
    onNodeStale(tx, nodeId);
}

//
//  REMOVE TX
//

void GraphEditor::removeNodeTx(Transaction& tx, NodeId id) {
    const Node* node = graph_.data().nodes().get(id);

    for (CrosswalkSeriesId csid : node->crosswalkSeries()) {
        removeCrosswalkSeriesTx(tx, csid);
    }

    for (EntryId eid : node->entries()) {
        removeEntryTx(tx, eid);
    }

    for (ExitId xid : node->exits()) {
        removeExitTx(tx, xid);
    }

    auto mut = graph_.removeNode(id);
    tx.recordUndo(std::move(mut.undo), bus_.nodeCreated, NodeCreated{id});
    tx.recordEvent(bus_.nodeRemoved, NodeRemoved{id});
    onNodeStale(tx, id);
}

void GraphEditor::removeCrosswalkSeriesTx(Transaction& tx,
                                          CrosswalkSeriesId id) {
    auto cs = graph_.data().crosswalkSeries().get(id);
    for (CrosswalkId cid : cs->crosswalks()) {
        removeCrosswalkTx(tx, cid);
    }
    auto mut = graph_.removeCrosswalkSeries(id);
    NodeId nodeId = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkSeriesCreated,
                  CrosswalkSeriesCreated{id, nodeId});
    tx.recordEvent(bus_.crosswalkSeriesRemoved,
                   CrosswalkSeriesRemoved{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::removeCrosswalkTx(Transaction& tx, CrosswalkId id) {
    auto mut = graph_.removeCrosswalk(id);
    NodeId nodeId = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.crosswalkCreated,
                  CrosswalkCreated{id, nodeId});
    tx.recordEvent(bus_.crosswalkRemoved, CrosswalkRemoved{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::removeEntryTx(Transaction& tx, EntryId id) {
    if (auto* e = graph_.data().entries().get(id)) {
        for (MovementId mid : e->movements()) {
            removeMovementTx(tx, mid);
        }
        setEntryLanesTx(tx, id, {});
    }
    auto mut = graph_.removeEntry(id);
    NodeId nodeId = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.entryCreated,
                  EntryCreated{id, nodeId});
    tx.recordEvent(bus_.entryRemoved, EntryRemoved{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::removeExitTx(Transaction& tx, ExitId id) {
    if (graph_.data().exits().get(id)) {
        setExitLanesTx(tx, id, {});
    }

    auto mut = graph_.removeExit(id);
    NodeId nodeId = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.exitCreated,
                  ExitCreated{id, nodeId});
    tx.recordEvent(bus_.exitRemoved, ExitRemoved{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::clearEntryLayoutTx(Transaction& tx, EntryId id) {
    auto* e = graph_.data().entries().get(id);
    auto clustersCopy = e->clusters();
    auto movementsCopy = e->movements();
    for (auto cId : clustersCopy) removeClusterTx(tx, cId);
    for (auto mId : movementsCopy) removeMovementTx(tx, mId);
}

//
//  PRIVATE TX
//

MovementId GraphEditor::createMovementTx(Transaction& tx, EntryId from,
                                         ExitId to,
                                         std::vector<EntryLaneId> lanes,
                                         std::vector<int> split) {
    auto mut =
        graph_.createMovement(from, to, std::move(lanes), std::move(split));
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.movementRemoved,
                  MovementRemoved{mut.id, nodeId});
    tx.recordEvent(bus_.movementCreated, MovementCreated{mut.id, nodeId});
    onNodeStale(tx, nodeId);
    return mut.id;
}

void GraphEditor::removeMovementTx(Transaction& tx, MovementId id) {
    auto mut = graph_.removeMovement(id);
    NodeId nodeId = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.movementCreated,
                  MovementCreated{id, nodeId});
    tx.recordEvent(bus_.movementRemoved, MovementRemoved{id, nodeId});
    onNodeStale(tx, nodeId);
}

ClusterId GraphEditor::createClusterTx(Transaction& tx, EntryId at,
                                       std::vector<MovementId> movements,
                                       std::vector<EntryLaneId> lanes) {
    auto mut = graph_.createCluster(at, std::move(movements), std::move(lanes));
    NodeId nodeId = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.clusterRemoved,
                  ClusterRemoved{mut.id, nodeId});
    tx.recordEvent(bus_.clusterCreated, ClusterCreated{mut.id, nodeId});
    onNodeStale(tx, nodeId);
    return mut.id;
}

void GraphEditor::removeClusterTx(Transaction& tx, ClusterId id) {
    auto mut = graph_.removeCluster(id);
    NodeId nodeId = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.clusterCreated,
                  ClusterCreated{id, nodeId});
    tx.recordEvent(bus_.clusterRemoved, ClusterRemoved{id, nodeId});
    onNodeStale(tx, nodeId);
}

void GraphEditor::setStreamIntergreenOverrideTx(Transaction& tx, NodeId node,
                                                StreamId evac, StreamId appr,
                                                int t_maxEvAp, int t_maxApEv) {
    auto mut = graph_.setStreamIntergreenOverride(node, evac, appr, t_maxEvAp,
                                                  t_maxApEv);
    tx.recordUndo(std::move(mut.undo), bus_.streamIntergreenOverrideChanged,
                  StreamIntergreenOverrideChanged{node});
    tx.recordEvent(bus_.streamIntergreenOverrideChanged,
                   StreamIntergreenOverrideChanged{node});
    onNodeStale(tx, node);
}

void GraphEditor::eraseStreamIntergreenOverrideTx(Transaction& tx, NodeId node,
                                                  StreamId evac,
                                                  StreamId appr) {
    auto mut = graph_.eraseStreamIntergreenOverride(node, evac, appr);
    tx.recordUndo(std::move(mut.undo), bus_.streamIntergreenOverrideChanged,
                  StreamIntergreenOverrideChanged{node});
    tx.recordEvent(bus_.streamIntergreenOverrideErased,
                   StreamIntergreenOverrideErased{node});
    onNodeStale(tx, node);
}

void GraphEditor::setStreamPermittedOverrideTx(Transaction& tx, NodeId node,
                                               StreamId evac, StreamId appr,
                                               bool permitted) {
    auto mut = graph_.setStreamPermittedOverride(node, evac, appr, permitted);
    tx.recordUndo(std::move(mut.undo), bus_.streamPermittedOverrideChanged,
                  StreamPermittedOverrideChanged{node});
    tx.recordEvent(bus_.streamPermittedOverrideChanged,
                   StreamPermittedOverrideChanged{node});
    onNodeStale(tx, node);
}

void GraphEditor::eraseStreamPermittedOverrideTx(Transaction& tx, NodeId node,
                                                 StreamId evac, StreamId appr) {
    auto mut = graph_.eraseStreamPermittedOverride(node, evac, appr);
    tx.recordUndo(std::move(mut.undo), bus_.streamPermittedOverrideChanged,
                  StreamPermittedOverrideChanged{node});
    tx.recordEvent(bus_.streamPermittedOverrideErased,
                   StreamPermittedOverrideErased{node});
    onNodeStale(tx, node);
}

void GraphEditor::onNodeStale(Transaction& tx, NodeId id) {
    tx.recordEvent(bus_.nodeRederived, NodeRederived{id});
    tx.recordUndoEvent(bus_.nodeRederived, NodeRederived{id});
    tx.addSolve(SolvePriority::GRAPH, [this] { graph_.solve(); });
}
