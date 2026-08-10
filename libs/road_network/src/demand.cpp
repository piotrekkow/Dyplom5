#include "road_network/demand.hpp"

#include "derived/demand_compute.hpp"
#include "road_network/graph/graph_data.hpp"

Mut<MovementId> Demand::setFlow(MovementId mov, NodeId at, Flow value) {
    auto& arr = data_.flows();
    return setSlotAt(arr, mov, at, value);
}

// Only called when the movement removed
Mut<MovementId> Demand::clearFlow(MovementId mov, NodeId at) {
    derived_.flowRatios().erase(mov);
    derived_.baseSaturations().erase(mov);
    auto& arr = data_.flows();
    return clearSlotAt(arr, mov, at);
}

Mut<MovementId> Demand::setSaturationOverride(MovementId mov, NodeId at,
                                              SaturationOverride value) {
    auto& arr = data_.saturationOverrides();
    return setSlotAt(arr, mov, at, value);
}
Mut<MovementId> Demand::clearSaturationOverride(MovementId mov, NodeId at) {
    auto& arr = data_.saturationOverrides();
    return clearSlotAt(arr, mov, at);
}

Demand::Rederived Demand::solve(const GraphData& graph) {
    Rederived out;
    for (auto id : dirty<EntryId>()) {
        evictDerived(id);
    }
    for (auto nid : dirty<NodeId>()) {
        const auto* node = graph.nodes().get(nid);
        if (!node) continue;
        for (auto id : node->entries()) {
            recomputeDerived(id, graph);
            out.entries.insert(id);
        }
    }
    dirty<NodeId>().clear();
    dirty<EntryId>().clear();
    return out;
}

void Demand::recomputeDerived(EntryId id, const GraphData& graph) {
    std::vector<Saturation> laneSats =
        demand_compute::entryLaneSaturations(graph, data_, regulations_, id);
    derived_.laneSaturations().set(id, std::move(laneSats));
    const auto& movs = graph.entries().get(id)->movements();
    for (auto mid : movs) {
        derived_.baseSaturations().set(
            mid,
            demand_compute::baseSaturation(graph, data_, regulations_, mid));
        derived_.flowRatios().set(mid,
                                  demand_compute::flowRatio(graph, *this, mid));
    }
}

void Demand::evictDerived(EntryId id) { derived_.laneSaturations().erase(id); }