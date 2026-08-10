#pragma once

#include "control/control_data.hpp"
#include "control/control_derived.hpp"
#include "road_network/util/ids.hpp"
#include "util/mutation_base.hpp"

struct Regulations;
class Signal;
class Graph;

class Control : public MutationBase<NodeId> {
   public:
    using Base = MutationBase<NodeId>;
    friend Base;

    explicit Control(const Regulations& regulations)
        : regulations_(regulations) {}

    const ControlData& data() const { return data_; }
    const ControlDerived& derived() const { return derived_; }

    EMut<TimingId, NodeId> createTiming(NodeId at, std::string name);
    EMut<TimingId, NodeId> setParams(TimingId id, Timing::Params p);
    EMut<TimingId, NodeId> removeTiming(TimingId id);

    EMut<StageId, NodeId> createStage(NodeId at,
                                      control::Stage<SignalGroupId> stage);
    EMut<StageId, NodeId> setParams(StageId id, Stage::Params p);
    EMut<StageId, NodeId> removeStage(StageId id);

    EMut<StageTransitionId, NodeId> createTransition(NodeId at, StageId from,
                                                     StageId to);
    EMut<StageTransitionId, NodeId> setParams(StageTransitionId id,
                                              StageTransition::Params p);
    EMut<StageTransitionId, NodeId> removeTransition(StageTransitionId id);

    struct Rederived {
        std::unordered_set<NodeId> nodes;
    };
    Rederived solve(const Signal& signal, const Graph& graph);

    NodeId parentOf(TimingId) const;
    NodeId parentOf(StageId) const;
    NodeId parentOf(StageTransitionId) const;

   private:
    ControlData data_;
    ControlDerived derived_;
    const Regulations& regulations_;

    void recomputeDerived(NodeId id, const Signal& signal);
    void evictDerived(NodeId id);

    void link(TimingId);
    void link(StageId);
    void link(StageTransitionId);
    void unlink(TimingId);
    void unlink(StageId);
    void unlink(StageTransitionId);
};