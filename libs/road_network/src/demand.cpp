#include "road_network/demand.hpp"

#include <optional>

#include "derived/demand_compute.hpp"
#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/graph/graph_data.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

template <typename Tag, typename T>
static std::optional<T> optCopy(const SlotArray<Tag, T>& arr, Id<Tag> id) {
    const T* p = arr.get(id);
    return p ? std::optional<T>(*p) : std::nullopt;
}

template <typename Tag, typename T>
static void restore(SlotArray<Tag, T>& arr, Id<Tag> id,
                    const std::optional<T>& old) {
    if (old)
        arr.set(id, *old);
    else
        arr.erase(id);
}

Demand::Mutation Demand::setFlow(MovementId id, Flow flow) {
    auto old = optCopy(data_.flows(), id);
    data_.flows().set(id, flow);
    dirty_.insert(id);
    return {.id = id,
            .undo = [this, id, old] { restore(data_.flows(), id, old); }};
}

Demand::Mutation Demand::eraseFlow(MovementId id) {
    auto old = optCopy(data_.flows(), id);
    data_.flows().erase(id);
    return {.id = id,
            .undo = [this, id, old] { restore(data_.flows(), id, old); }};
}

Demand::Mutation Demand::setSaturationOverride(MovementId id,
                                               SaturationOverride override) {
    auto old = optCopy(data_.saturationOverrides(), id);
    data_.saturationOverrides().set(id, override);
    dirty_.insert(id);
    return {.id = id, .undo = [this, id, old] {
                restore(data_.saturationOverrides(), id, old);
            }};
}
Demand::Mutation Demand::eraseSaturationOverride(MovementId id) {
    auto old = optCopy(data_.saturationOverrides(), id);
    data_.saturationOverrides().erase(id);
    return {.id = id, .undo = [this, id, old] {
                restore(data_.saturationOverrides(), id, old);
            }};
}

void Demand::solve(const GraphData& graph) {
    std::unordered_set<EntryId> dirtyEntries;
    for (MovementId mId : dirty_) {
        const auto* m = graph.movements().get(mId);
        if (!m) continue;
        EntryId eId = m->from();
        dirtyEntries.insert(eId);
        if (auto opp = graph_geometry::findOpposingEntry(graph, eId))
            dirtyEntries.insert(*opp);
    }
    for (EntryId id : dirtyEntries) recomputeDerived(graph, id);
    dirty_.clear();
}

void Demand::recomputeDerived(const GraphData& graph, EntryId id) {
    auto eMap = demand_compute::entryLaneSaturation(graph, data_, id);
    for (auto& [lane, sat] : eMap) {
        derived_.laneSaturations().set(lane, sat);
    }
}