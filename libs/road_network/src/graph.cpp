#include "road_network/graph.hpp"

#include <cassert>
#include <utility>
#include <vector>

#include "derived/graph_compute.hpp"
#include "road_network/graph/data/movement.hpp"
#include "road_network/graph/data/stream_type.hpp"
#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/util/geometry/position.hpp"
#include "road_network/util/geometry/vector2.hpp"
#include "road_network/util/id.hpp"
#include "road_network/util/ids.hpp"

namespace {

// Always store override pairs with the smaller StreamId first so that
// set(a,b) and set(b,a) alias to the same entry, and erase(b,a) clears
// what set(a,b) stored.
std::pair<StreamId, StreamId> canonicalPair(StreamId a, StreamId b) {
    if (a.index() != b.index())
        return a.index() < b.index() ? std::pair{a, b} : std::pair{b, a};
    // same variant type — compare underlying Id value
    bool aFirst = std::visit(
        [&](auto aid) { return aid < std::get<decltype(aid)>(b); }, a);
    return aFirst ? std::pair{a, b} : std::pair{b, a};
}

void applyStreamIntergreenOverrides(
    StreamIntergreenMap& sim, const StreamIntergreenOverrideMap& overrides) {
    for (const auto& [evac, apprMap] : overrides)
        for (const auto& [appr, t_max] : apprMap)
            sim[evac][appr].t_max = t_max;
}

void applyStreamPermittedOverrides(
    StreamPermittedMap& spm, const StreamPermittedOverrideMap& overrides) {
    for (const auto& [evac, apprMap] : overrides) {
        for (const auto& [appr, permitted] : apprMap) {
            if (permitted) {
                spm[evac].insert(appr);
                spm[appr].insert(evac);
            } else {
                if (auto it = spm.find(evac); it != spm.end())
                    it->second.erase(appr);
                if (auto it = spm.find(appr); it != spm.end())
                    it->second.erase(evac);
            }
        }
    }
}
}  // namespace

//
//  NODE
//

Graph::NodeMutation Graph::createNode(Position center) {
    auto id = data_.nodes().emplace(center);
    dirty_.insert(id);
    return {.id = id,
            .undo = [this, id] { data_.nodes().release(removeNodeImpl(id)); }};
}

Graph::NodeMutation Graph::removeNode(NodeId id) {
    assert(data_.nodes().alive(id));
    auto vs = removeNodeImpl(id);
    return {.id = id, .undo = [this, vs = std::move(vs)] mutable {
                createNodeImpl(std::move(vs));
            }};
}

//
//  ENTRY
//

Graph::EntityMutation<EntryId> Graph::createEntry(NodeId at, Position position,
                                                  Vector2 heading) {
    auto id = data_.entries().emplace(at, position, heading);
    data_.nodes().get(at)->entries().push_back(id);
    dirty_.insert(at);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.entries().release(removeEntryImpl(id));
            }};
}

Graph::EntityMutation<EntryId> Graph::setEntrySpeedLimit(EntryId id,
                                                         float speedLimit) {
    assert(data_.entries().alive(id));
    auto* e = data_.entries().get(id);
    NodeId nodeId = e->at();
    float old = e->speedLimit();
    auto apply = [this, id, nodeId](float v) {
        data_.entries().get(id)->setSpeedLimit(v);
        dirty_.insert(nodeId);
    };
    apply(speedLimit);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}

Graph::EntityMutation<EntryId> Graph::setEntryPosition(EntryId id,
                                                       Position position) {
    assert(data_.entries().alive(id));
    auto* e = data_.entries().get(id);
    NodeId nodeId = e->at();
    auto old = e->position();
    auto apply = [this, id, nodeId](Position v) {
        data_.entries().get(id)->setPosition(v);
        dirty_.insert(nodeId);
    };
    apply(position);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}

Graph::EntityMutation<EntryId> Graph::setEntryHeading(EntryId id,
                                                      Vector2 heading) {
    assert(data_.entries().alive(id));
    auto* e = data_.entries().get(id);
    NodeId nodeId = e->at();
    auto old = e->heading();
    auto apply = [this, id, nodeId](Vector2 v) {
        data_.entries().get(id)->setHeading(v);
        dirty_.insert(nodeId);
    };
    apply(heading);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}

Graph::NodeMutation Graph::removeEntry(EntryId id) {
    assert(data_.entries().alive(id));
    auto vs = removeEntryImpl(id);
    return {.id = vs.value.at(), .undo = [this, vs = std::move(vs)] mutable {
                createEntryImpl(std::move(vs));
            }};
}

//
//  EXIT
//

Graph::EntityMutation<ExitId> Graph::createExit(NodeId at, Position position,
                                                Vector2 heading) {
    auto id = data_.exits().emplace(at, position, heading);
    data_.nodes().get(at)->exits().push_back(id);
    dirty_.insert(at);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.exits().release(removeExitImpl(id));
            }};
}
Graph::EntityMutation<ExitId> Graph::setExitSpeedLimit(ExitId id,
                                                       float speedLimit) {
    assert(data_.exits().alive(id));
    auto* x = data_.exits().get(id);
    NodeId nodeId = x->at();
    float old = x->speedLimit();
    auto apply = [this, id, nodeId](float v) {
        data_.exits().get(id)->setSpeedLimit(v);
        dirty_.insert(nodeId);
    };
    apply(speedLimit);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<ExitId> Graph::setExitPosition(ExitId id,
                                                     Position position) {
    assert(data_.exits().alive(id));
    auto* x = data_.exits().get(id);
    NodeId nodeId = x->at();
    auto old = x->position();
    auto apply = [this, id, nodeId](Position v) {
        data_.exits().get(id)->setPosition(v);
        dirty_.insert(nodeId);
    };
    apply(position);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<ExitId> Graph::setExitHeading(ExitId id,
                                                    Vector2 heading) {
    assert(data_.exits().alive(id));
    auto* x = data_.exits().get(id);
    NodeId nodeId = x->at();
    auto old = x->heading();
    auto apply = [this, id, nodeId](Vector2 v) {
        data_.exits().get(id)->setHeading(v);
        dirty_.insert(nodeId);
    };
    apply(heading);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::NodeMutation Graph::removeExit(ExitId id) {
    assert(data_.exits().alive(id));
    auto vs = removeExitImpl(id);
    return {.id = vs.value.at(), .undo = [this, vs = std::move(vs)] mutable {
                createExitImpl(std::move(vs));
            }};
}

//
//  ENTRY LANE
//

Graph::EntityMutation<EntryId> Graph::setEntryLanes(
    EntryId entryId, std::vector<EntryLaneSpec> specs) {
    assert(data_.entries().alive(entryId));
    auto* e = data_.entries().get(entryId);
    NodeId nodeId = e->at();

    auto& currentIds = e->lanes();
    const size_t oldCount = currentIds.size();
    const size_t newCount = specs.size();
    const size_t kept = std::min(oldCount, newCount);

    std::vector<EntryLane> oldData;
    oldData.reserve(oldCount);
    for (EntryLaneId lid : currentIds)
        oldData.push_back(*data_.entryLanes().get(lid));

    using LaneVS = SlotMap<EntryLaneTag, EntryLane>::VacatedSlot;
    std::vector<LaneVS> removed;
    removed.reserve(oldCount > newCount ? oldCount - newCount : 0);
    for (size_t i = newCount; i < oldCount; ++i)
        removed.push_back(removeEntryLaneImpl(currentIds[i]));
    currentIds.erase(currentIds.begin() + static_cast<std::ptrdiff_t>(kept),
                     currentIds.end());

    for (size_t i = 0; i < kept; ++i) {
        auto* l = data_.entryLanes().get(currentIds[i]);
        l->setWidth(specs[i].width);
        l->setStopLineOffset(specs[i].stopLineOffset);
        l->setStorageLength(specs[i].storageLength);
    }

    std::vector<EntryLaneId> added;
    added.reserve(newCount > oldCount ? newCount - oldCount : 0);
    for (size_t i = oldCount; i < newCount; ++i) {
        EntryLaneId id = data_.entryLanes().emplace(entryId);
        auto* l = data_.entryLanes().get(id);
        l->setWidth(specs[i].width);
        l->setStopLineOffset(specs[i].stopLineOffset);
        l->setStorageLength(specs[i].storageLength);
        currentIds.push_back(id);
        added.push_back(id);
    }

    dirty_.insert(nodeId);

    return {.id = entryId,
            .at = nodeId,
            .undo = [this, entryId, nodeId, kept, oldData = std::move(oldData),
                     added = std::move(added),
                     removed = std::move(removed)]() mutable {
                for (EntryLaneId id : added)
                    data_.entryLanes().release(removeEntryLaneImpl(id));

                auto& lanes = data_.entries().get(entryId)->lanes();
                lanes.erase(lanes.begin() + static_cast<std::ptrdiff_t>(kept),
                            lanes.end());
                for (size_t i = 0; i < kept; ++i)
                    *data_.entryLanes().get(lanes[i]) = oldData[i];

                for (auto& vs : removed) {
                    lanes.push_back(vs.id);
                    data_.entryLanes().reinsert(std::move(vs));
                }

                dirty_.insert(nodeId);
            }};
}

//
//  EXIT LANE
//

Graph::EntityMutation<ExitId> Graph::setExitLanes(
    ExitId exitId, std::vector<ExitLaneSpec> specs) {
    assert(data_.exits().alive(exitId));
    auto* x = data_.exits().get(exitId);
    NodeId nodeId = x->at();

    auto& currentIds = x->lanes();
    const size_t oldCount = currentIds.size();
    const size_t newCount = specs.size();
    const size_t kept = std::min(oldCount, newCount);

    std::vector<ExitLane> oldData;
    oldData.reserve(oldCount);
    for (ExitLaneId lid : currentIds)
        oldData.push_back(*data_.exitLanes().get(lid));

    using LaneVS = SlotMap<ExitLaneTag, ExitLane>::VacatedSlot;
    std::vector<LaneVS> removed;
    removed.reserve(oldCount > newCount ? oldCount - newCount : 0);
    for (size_t i = newCount; i < oldCount; ++i)
        removed.push_back(removeExitLaneImpl(currentIds[i]));
    currentIds.erase(currentIds.begin() + static_cast<std::ptrdiff_t>(kept),
                     currentIds.end());

    for (size_t i = 0; i < kept; ++i) {
        auto* l = data_.exitLanes().get(currentIds[i]);
        l->setWidth(specs[i].width);
    }

    std::vector<ExitLaneId> added;
    added.reserve(newCount > oldCount ? newCount - oldCount : 0);
    for (size_t i = oldCount; i < newCount; ++i) {
        ExitLaneId id = data_.exitLanes().emplace(exitId);
        data_.exitLanes().get(id)->setWidth(specs[i].width);
        currentIds.push_back(id);
        added.push_back(id);
    }

    dirty_.insert(nodeId);

    return {.id = exitId,
            .at = nodeId,
            .undo = [this, exitId, nodeId, kept, oldData = std::move(oldData),
                     added = std::move(added),
                     removed = std::move(removed)]() mutable {
                for (ExitLaneId id : added)
                    data_.exitLanes().release(removeExitLaneImpl(id));

                auto& lanes = data_.exits().get(exitId)->lanes();
                lanes.erase(lanes.begin() + static_cast<std::ptrdiff_t>(kept),
                            lanes.end());
                for (size_t i = 0; i < kept; ++i)
                    *data_.exitLanes().get(lanes[i]) = oldData[i];

                for (auto& vs : removed) {
                    lanes.push_back(vs.id);
                    data_.exitLanes().reinsert(std::move(vs));
                }

                dirty_.insert(nodeId);
            }};
}

//
//  CROSSWALK SERIES
//

Graph::EntityMutation<CrosswalkSeriesId> Graph::createCrosswalkSeries(
    NodeId at) {
    auto id = data_.crosswalkSeries().emplace(at);
    data_.nodes().get(at)->crosswalkSeries().push_back(id);
    dirty_.insert(at);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.crosswalkSeries().release(removeCrosswalkSeriesImpl(id));
            }};
}

Graph::EntityMutation<CrosswalkSeriesId>
Graph::setCrosswalkSeriesPedestrianPath(CrosswalkSeriesId id, Polyline path) {
    assert(data_.crosswalkSeries().alive(id));
    auto* cs = data_.crosswalkSeries().get(id);
    NodeId nodeId = cs->at();
    auto old = cs->pedestrianPath();
    auto apply = [this, id, nodeId](Polyline v) {
        data_.crosswalkSeries().get(id)->pedestrianPath() = std::move(v);
        dirty_.insert(nodeId);
    };
    apply(std::move(path));
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}

Graph::NodeMutation Graph::removeCrosswalkSeries(CrosswalkSeriesId id) {
    assert(data_.crosswalkSeries().alive(id));
    auto vs = removeCrosswalkSeriesImpl(id);
    auto nodeId = vs.value.at();
    return {.id = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createCrosswalkSeriesImpl(std::move(vs));
            }};
}

//
//  CROSSWALK
//

Graph::EntityMutation<CrosswalkId> Graph::createCrosswalk(CrosswalkSeriesId at,
                                                          Line line) {
    auto id = data_.crosswalks().emplace(at, line);
    data_.crosswalkSeries().get(at)->crosswalks().push_back(id);
    auto nodeId = data_.crosswalkSeries().get(at)->at();
    dirty_.insert(nodeId);
    return {.id = id, .at = nodeId, .undo = [this, id] {
                data_.crosswalks().release(removeCrosswalkImpl(id));
            }};
}
Graph::EntityMutation<CrosswalkId> Graph::setCrosswalkLine(CrosswalkId id,
                                                           Line line) {
    assert(data_.crosswalks().alive(id));
    auto* c = data_.crosswalks().get(id);
    NodeId nodeId = data_.crosswalkSeries().get(c->parent())->at();
    auto old = c->params().line;
    auto apply = [this, id, nodeId](Line v) {
        data_.crosswalks().get(id)->setLine(v);
        dirty_.insert(nodeId);
    };
    apply(line);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<CrosswalkId> Graph::setCrosswalkEdge(
    CrosswalkId id, Crosswalk::SegmentParams edge) {
    assert(data_.crosswalks().alive(id));
    auto* c = data_.crosswalks().get(id);
    NodeId nodeId = data_.crosswalkSeries().get(c->parent())->at();
    auto old = c->params().edge;
    auto apply = [this, id, nodeId](Crosswalk::SegmentParams v) {
        data_.crosswalks().get(id)->setEdge(v);
        dirty_.insert(nodeId);
    };
    apply(edge);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<CrosswalkId> Graph::setCrosswalkSplit(
    CrosswalkId id, Crosswalk::SegmentParams split) {
    assert(data_.crosswalks().alive(id));
    auto* c = data_.crosswalks().get(id);
    NodeId nodeId = data_.crosswalkSeries().get(c->parent())->at();
    auto old = c->params().split;
    auto apply = [this, id, nodeId](Crosswalk::SegmentParams v) {
        data_.crosswalks().get(id)->setSplit(v);
        dirty_.insert(nodeId);
    };
    apply(split);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<CrosswalkId> Graph::setCrosswalkType(CrosswalkId id,
                                                           CrosswalkType type) {
    assert(data_.crosswalks().alive(id));
    auto* c = data_.crosswalks().get(id);
    NodeId nodeId = data_.crosswalkSeries().get(c->parent())->at();
    auto old = c->type();
    auto apply = [this, id, nodeId](CrosswalkType v) {
        data_.crosswalks().get(id)->setType(v);
        dirty_.insert(nodeId);
    };
    apply(type);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::NodeMutation Graph::removeCrosswalk(CrosswalkId id) {
    assert(data_.crosswalks().alive(id));
    auto vs = removeCrosswalkImpl(id);
    auto nodeId = data_.crosswalkSeries().get(vs.value.parent())->at();
    return {.id = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createCrosswalkImpl(std::move(vs));
            }};
}

//
//  MOVEMENT
//

Graph::EntityMutation<MovementId> Graph::createMovement(
    EntryId from, ExitId to, std::vector<EntryLaneId> lanes,
    std::vector<int> split) {
    auto id =
        data_.movements().emplace(from, to, std::move(lanes), std::move(split));
    // from->at() == to->at() check performed in editor
    auto* entry = data_.entries().get(from);
    entry->movements().push_back(id);
    auto nodeId = entry->at();
    dirty_.insert(nodeId);
    return {.id = id, .at = nodeId, .undo = [this, id] {
                data_.movements().release(removeMovementImpl(id));
            }};
}
Graph::EntityMutation<MovementId> Graph::setMovementSplits(
    MovementId id, std::vector<int> splits) {
    assert(data_.movements().alive(id));
    auto* m = data_.movements().get(id);
    NodeId nodeId = data_.entries().get(m->from())->at();
    auto old = m->splitPerLane();
    auto apply = [this, id, nodeId](std::vector<int> v) {
        data_.movements().get(id)->setSplits(std::move(v));
        dirty_.insert(nodeId);
    };
    apply(std::move(splits));
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<MovementId> Graph::setMovementType(MovementId id,
                                                         MovementType type) {
    assert(data_.movements().alive(id));
    auto* m = data_.movements().get(id);
    NodeId nodeId = data_.entries().get(m->from())->at();
    auto old = m->type();
    auto apply = [this, id, nodeId](MovementType v) {
        data_.movements().get(id)->setType(v);
        dirty_.insert(nodeId);
    };
    apply(type);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::EntityMutation<MovementId> Graph::setMovementPathSpec(
    MovementId id, MovementPathSpec spec) {
    assert(data_.movements().alive(id));
    auto* m = data_.movements().get(id);
    NodeId nodeId = data_.entries().get(m->from())->at();
    auto old = m->pathSpec();
    auto apply = [this, id, nodeId](MovementPathSpec v) {
        data_.movements().get(id)->setPathSpec(v);
        dirty_.insert(nodeId);
    };
    apply(spec);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}
Graph::NodeMutation Graph::removeMovement(MovementId id) {
    assert(data_.movements().alive(id));
    auto vs = removeMovementImpl(id);
    auto nodeId = data_.entries().get(vs.value.from())->at();
    return {.id = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createMovementImpl(std::move(vs));
            }};
}

//
//  CLUSTER
//

Graph::EntityMutation<ClusterId> Graph::createCluster(
    EntryId at, std::vector<MovementId> movements,
    std::vector<EntryLaneId> lanes) {
    auto id =
        data_.clusters().emplace(at, std::move(movements), std::move(lanes));
    // from->at() == to->at() check performed in editor
    auto* entry = data_.entries().get(at);
    entry->clusters().push_back(id);
    auto nodeId = entry->at();
    dirty_.insert(nodeId);
    return {.id = id, .at = nodeId, .undo = [this, id] {
                data_.clusters().release(removeClusterImpl(id));
            }};
}
Graph::NodeMutation Graph::removeCluster(ClusterId id) {
    assert(data_.clusters().alive(id));
    auto vs = removeClusterImpl(id);
    auto nodeId = data_.entries().get(vs.value.at())->at();
    return {.id = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createClusterImpl(std::move(vs));
            }};
}

//
//  IMPLEMENTATION HELPERS
//

SlotMap<NodeTag, Node>::VacatedSlot Graph::removeNodeImpl(NodeId id) {
    auto vs = data_.nodes().vacate(id);
    derived_.conflict().erase(id);
    derived_.streamIntergreen().erase(id);
    dirty_.erase(id);
    return vs;
}

void Graph::createNodeImpl(SlotMap<NodeTag, Node>::VacatedSlot vs) {
    NodeId id = vs.id;
    data_.nodes().reinsert(std::move(vs));
    dirty_.insert(id);
}

SlotMap<EntryTag, Entry>::VacatedSlot Graph::removeEntryImpl(EntryId id) {
    auto vs = data_.entries().vacate(id);
    auto& entries = data_.nodes().get(vs.value.at())->entries();
    std::erase(entries, id);
    dirty_.insert(vs.value.at());
    return vs;
}

void Graph::createEntryImpl(SlotMap<EntryTag, Entry>::VacatedSlot vs) {
    NodeId nodeId = vs.value.at();
    EntryId id = vs.id;
    data_.entries().reinsert(std::move(vs));
    data_.nodes().get(nodeId)->entries().push_back(id);
    dirty_.insert(nodeId);
}

SlotMap<ExitTag, Exit>::VacatedSlot Graph::removeExitImpl(ExitId id) {
    auto vs = data_.exits().vacate(id);
    std::erase(data_.nodes().get(vs.value.at())->exits(), id);
    dirty_.insert(vs.value.at());
    return vs;
}

void Graph::createExitImpl(SlotMap<ExitTag, Exit>::VacatedSlot vs) {
    NodeId nodeId = vs.value.at();
    ExitId exitId = vs.id;
    data_.exits().reinsert(std::move(vs));
    data_.nodes().get(nodeId)->exits().push_back(exitId);
    dirty_.insert(nodeId);
}

SlotMap<MovementTag, Movement>::VacatedSlot Graph::removeMovementImpl(
    MovementId id) {
    auto vs = data_.movements().vacate(id);
    derived_.turnTypes().erase(id);
    std::erase(data_.entries().get(vs.value.from())->movements(), id);
    dirty_.insert(data_.entries().get(vs.value.from())->at());
    return vs;
}

void Graph::createMovementImpl(SlotMap<MovementTag, Movement>::VacatedSlot vs) {
    EntryId entryId = vs.value.from();
    MovementId movId = vs.id;
    NodeId nodeId = data_.entries().get(entryId)->at();
    data_.movements().reinsert(std::move(vs));
    data_.entries().get(entryId)->movements().push_back(movId);
    dirty_.insert(nodeId);
}

SlotMap<EntryLaneTag, EntryLane>::VacatedSlot Graph::removeEntryLaneImpl(
    EntryLaneId id) {
    auto vs = data_.entryLanes().vacate(id);
    dirty_.insert(data_.entries().get(vs.value.at())->at());
    return vs;
}

void Graph::createEntryLaneImpl(
    SlotMap<EntryLaneTag, EntryLane>::VacatedSlot vs) {
    NodeId nodeId = data_.entries().get(vs.value.at())->at();
    data_.entryLanes().reinsert(std::move(vs));
    dirty_.insert(nodeId);
}

SlotMap<ExitLaneTag, ExitLane>::VacatedSlot Graph::removeExitLaneImpl(
    ExitLaneId id) {
    auto vs = data_.exitLanes().vacate(id);
    dirty_.insert(data_.exits().get(vs.value.at())->at());
    return vs;
}

void Graph::createExitLaneImpl(SlotMap<ExitLaneTag, ExitLane>::VacatedSlot vs) {
    NodeId nodeId = data_.exits().get(vs.value.at())->at();
    data_.exitLanes().reinsert(std::move(vs));
    dirty_.insert(nodeId);
}

SlotMap<CrosswalkTag, Crosswalk>::VacatedSlot Graph::removeCrosswalkImpl(
    CrosswalkId id) {
    auto vs = data_.crosswalks().vacate(id);
    std::erase(data_.crosswalkSeries().get(vs.value.parent())->crosswalks(),
               id);
    NodeId nodeId = data_.crosswalkSeries().get(vs.value.parent())->at();
    dirty_.insert(nodeId);
    return vs;
}

void Graph::createCrosswalkImpl(
    SlotMap<CrosswalkTag, Crosswalk>::VacatedSlot vs) {
    CrosswalkSeriesId parentId = vs.value.parent();
    CrosswalkId cwId = vs.id;
    NodeId nodeId = data_.crosswalkSeries().get(parentId)->at();
    data_.crosswalks().reinsert(std::move(vs));
    data_.crosswalkSeries().get(parentId)->crosswalks().push_back(cwId);
    dirty_.insert(nodeId);
}

SlotMap<CrosswalkSeriesTag, CrosswalkSeries>::VacatedSlot
Graph::removeCrosswalkSeriesImpl(CrosswalkSeriesId id) {
    auto vs = data_.crosswalkSeries().vacate(id);
    NodeId nodeId = vs.value.at();
    std::erase(data_.nodes().get(nodeId)->crosswalkSeries(), id);
    dirty_.insert(nodeId);
    return vs;
}

void Graph::createCrosswalkSeriesImpl(
    SlotMap<CrosswalkSeriesTag, CrosswalkSeries>::VacatedSlot vs) {
    NodeId nodeId = vs.value.at();
    CrosswalkSeriesId csId = vs.id;
    data_.crosswalkSeries().reinsert(std::move(vs));
    data_.nodes().get(nodeId)->crosswalkSeries().push_back(csId);
    dirty_.insert(nodeId);
}

SlotMap<ClusterTag, Cluster>::VacatedSlot Graph::removeClusterImpl(
    ClusterId id) {
    auto vs = data_.clusters().vacate(id);
    std::erase(data_.entries().get(vs.value.at())->clusters(), id);
    NodeId nodeId = data_.entries().get(vs.value.at())->at();
    dirty_.insert(nodeId);
    return vs;
}

void Graph::createClusterImpl(SlotMap<ClusterTag, Cluster>::VacatedSlot vs) {
    EntryId entryId = vs.value.at();
    ClusterId clusterId = vs.id;
    NodeId nodeId = data_.entries().get(entryId)->at();
    data_.clusters().reinsert(std::move(vs));
    data_.entries().get(entryId)->clusters().push_back(clusterId);
    dirty_.insert(nodeId);
}

//
//  STREAM PERMITTED OVERRIDE
//

Graph::NodeMutation Graph::setStreamPermittedOverride(NodeId node,
                                                      StreamId evac,
                                                      StreamId appr,
                                                      bool permitted) {
    assert(data_.nodes().alive(node));
    auto [a, b] = canonicalPair(evac, appr);

    std::optional<bool> old;
    if (const auto* m = data_.streamPermittedOverrides().get(node)) {
        if (auto it1 = m->find(a); it1 != m->end()) {
            if (auto it2 = it1->second.find(b); it2 != it1->second.end())
                old = it2->second;
        }
    }

    if (!data_.streamPermittedOverrides().get(node))
        data_.streamPermittedOverrides().set(node, {});
    (*data_.streamPermittedOverrides().get(node))[a][b] = permitted;
    dirty_.insert(node);

    return {.id = node, .undo = [this, node, a, b, old] {
                if (auto* m = data_.streamPermittedOverrides().get(node)) {
                    if (old) {
                        (*m)[a][b] = *old;
                    } else {
                        if (auto it = m->find(a); it != m->end()) {
                            it->second.erase(b);
                            if (it->second.empty()) m->erase(it);
                        }
                    }
                }
                dirty_.insert(node);
            }};
}

Graph::NodeMutation Graph::eraseStreamPermittedOverride(NodeId node,
                                                        StreamId evac,
                                                        StreamId appr) {
    assert(data_.nodes().alive(node));
    auto [a, b] = canonicalPair(evac, appr);

    std::optional<bool> old;
    if (auto* m = data_.streamPermittedOverrides().get(node)) {
        if (auto it1 = m->find(a); it1 != m->end()) {
            if (auto it2 = it1->second.find(b); it2 != it1->second.end()) {
                old = it2->second;
                it1->second.erase(it2);
                if (it1->second.empty()) m->erase(it1);
            }
        }
    }
    dirty_.insert(node);

    return {.id = node, .undo = [this, node, a, b, old] {
                if (old) {
                    if (!data_.streamPermittedOverrides().get(node))
                        data_.streamPermittedOverrides().set(node, {});
                    (*data_.streamPermittedOverrides().get(node))[a][b] = *old;
                }
                dirty_.insert(node);
            }};
}

//
//  STREAM INTERGREEN OVERRIDE
//

Graph::NodeMutation Graph::setStreamIntergreenOverride(
    NodeId node, StreamId evac, StreamId appr, int t_maxEvAp, int t_maxApEv) {
    assert(data_.nodes().alive(node));

    auto getOld = [&](StreamId a, StreamId b) -> std::optional<int> {
        if (const auto* m = data_.streamIntergreenOverrides().get(node)) {
            if (auto it1 = m->find(a); it1 != m->end()) {
                if (auto it2 = it1->second.find(b); it2 != it1->second.end())
                    return it2->second;
            }
        }
        return std::nullopt;
    };
    auto old_ab = getOld(evac, appr);
    auto old_ba = getOld(appr, evac);

    if (!data_.streamIntergreenOverrides().get(node))
        data_.streamIntergreenOverrides().set(node, {});
    auto& m = *data_.streamIntergreenOverrides().get(node);
    m[evac][appr] = t_maxEvAp;
    m[appr][evac] = t_maxApEv;
    dirty_.insert(node);

    return {.id = node, .undo = [this, node, evac, appr, old_ab, old_ba] {
                auto* m = data_.streamIntergreenOverrides().get(node);
                if (!m) return;
                auto restore = [&](StreamId a, StreamId b,
                                   const std::optional<int>& old) {
                    if (old) {
                        (*m)[a][b] = *old;
                    } else {
                        if (auto it = m->find(a); it != m->end()) {
                            it->second.erase(b);
                            if (it->second.empty()) m->erase(it);
                        }
                    }
                };
                restore(evac, appr, old_ab);
                restore(appr, evac, old_ba);
                dirty_.insert(node);
            }};
}

Graph::NodeMutation Graph::eraseStreamIntergreenOverride(NodeId node,
                                                         StreamId evac,
                                                         StreamId appr) {
    assert(data_.nodes().alive(node));

    auto getAndErase = [&](StreamId a, StreamId b) -> std::optional<int> {
        if (auto* m = data_.streamIntergreenOverrides().get(node)) {
            if (auto it1 = m->find(a); it1 != m->end()) {
                if (auto it2 = it1->second.find(b); it2 != it1->second.end()) {
                    auto old = it2->second;
                    it1->second.erase(it2);
                    if (it1->second.empty()) m->erase(it1);
                    return old;
                }
            }
        }
        return std::nullopt;
    };
    auto old_ab = getAndErase(evac, appr);
    auto old_ba = getAndErase(appr, evac);
    dirty_.insert(node);

    return {.id = node, .undo = [this, node, evac, appr, old_ab, old_ba] {
                if (old_ab || old_ba) {
                    if (!data_.streamIntergreenOverrides().get(node))
                        data_.streamIntergreenOverrides().set(node, {});
                    auto& m = *data_.streamIntergreenOverrides().get(node);
                    if (old_ab) m[evac][appr] = *old_ab;
                    if (old_ba) m[appr][evac] = *old_ba;
                }
                dirty_.insert(node);
            }};
}

//
//
//

void Graph::recomputeDerived(NodeId id) {
    derived_.conflict().set(id, graph_compute::conflictMap(data_, id));
    auto sim = graph_compute::streamIntergreenMap(*this, id);
    if (const auto* overrides = data_.streamIntergreenOverrides().get(id))
        applyStreamIntergreenOverrides(sim, *overrides);
    derived_.streamIntergreen().set(id, std::move(sim));

    auto spm = graph_compute::streamPermittedMap(*this, id);
    if (const auto* overrides = data_.streamPermittedOverrides().get(id))
        applyStreamPermittedOverrides(spm, *overrides);
    derived_.streamPermitted().set(id, std::move(spm));

    for (const auto& entry : data_.nodes().get(id)->entries()) {
        for (const auto& movement : data_.entries().get(entry)->movements())
            derived_.turnTypes().set(
                movement, graph_geometry::classifyTurn(data_, movement));
    }
}