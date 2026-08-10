#pragma once

#include <regulations/regulations.hpp>

#include "demand/demand_data.hpp"
#include "demand/demand_derived.hpp"
#include "util/mutation_base.hpp"

class GraphData;

class Demand : public MutationBase<NodeId, EntryId> {
   public:
    explicit Demand(const Regulations& regulations)
        : regulations_(regulations) {}
    const DemandData& data() const { return data_; }
    const DemandDerived& derived() const { return derived_; }

    Mut<MovementId> setFlow(MovementId mov, NodeId at, Flow value);
    Mut<MovementId> clearFlow(MovementId mov, NodeId at);

    Mut<MovementId> setSaturationOverride(MovementId mov, NodeId at,
                                          SaturationOverride value);
    Mut<MovementId> clearSaturationOverride(MovementId mov, NodeId at);

    struct Rederived {
        std::unordered_set<EntryId> entries;
    };
    Rederived solve(const GraphData& graph);

   private:
    DemandData data_;
    DemandDerived derived_;
    const Regulations& regulations_;

    void recomputeDerived(EntryId id, const GraphData& graph);
    void evictDerived(EntryId id);
};
