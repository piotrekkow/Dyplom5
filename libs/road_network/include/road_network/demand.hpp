#pragma once

#include <unordered_set>

#include "demand/demand_data.hpp"
#include "demand/demand_derived.hpp"
#include "road_network/demand/data/saturation_override.hpp"
#include "util/ids.hpp"
#include <functional>

class GraphData;

class Demand {
   public:
    const DemandData& data() const { return data_; }
    const DemandDerived& derived() const { return derived_; }

    // GraphData required for dirty Movement -> Entry + opposing Entry
    // conversion
    void solve(const GraphData& graph);

    struct Mutation {
        MovementId id;
        std::move_only_function<void()> undo;
    };

    Mutation setFlow(MovementId id, Flow flow);
    Mutation eraseFlow(MovementId id);

    Mutation setSaturationOverride(MovementId id, SaturationOverride override);
    Mutation eraseSaturationOverride(MovementId id);

   private:
    DemandData data_;
    DemandDerived derived_;

    std::unordered_set<MovementId> dirty_;
    void recomputeDerived(const GraphData& graph, EntryId id);
};
