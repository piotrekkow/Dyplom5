#pragma once

#include "signal/signal_data.hpp"
#include "signal/signal_derived.hpp"
#include "util/ids.hpp"
#include "util/mutation_base.hpp"

class Graph;
struct Regulations;

class Signal : public MutationBase<NodeId> {
    using Base = MutationBase<NodeId>;
    friend Base;

   public:
    explicit Signal(const Regulations& regulations)
        : regulations_(regulations) {}

    const SignalData& data() const { return data_; }
    const SignalDerived& derived() const { return derived_; }

    EMut<SignalGroupId, NodeId> createMovementGroup(
        NodeId node, EntryId entry, std::vector<ClusterId> clusters,
        std::vector<MovementId> movements, int minIntervalFixed);
    EMut<SignalGroupId, NodeId> createCrosswalkGroup(
        NodeId node, std::vector<CrosswalkId> crosswalks, SignalKind kind,
        int minIntervalFixed);

    EMut<SignalGroupId, NodeId> setParams(SignalGroupId id,
                                          MovementSignalGroup::Params p);
    EMut<SignalGroupId, NodeId> setParams(SignalGroupId id,
                                          CrosswalkSignalGroup::Params p);

    EMut<SignalGroupId, NodeId> removeSignalGroup(SignalGroupId id);

    struct Rederived {
        std::unordered_set<NodeId> nodes;
    };
    Rederived solve(const Graph& graph);

    [[nodiscard]] NodeId parentOf(SignalGroupId id) const;

   private:
    SignalData data_;
    SignalDerived derived_;
    const Regulations& regulations_;

    void recomputeDerived(const Graph& graph, NodeId id);
    void evictDerived(NodeId id);

    EMut<SignalGroupId, NodeId> createSignalGroup(SignalGroup sg);
    template <typename Group>
    EMut<SignalGroupId, NodeId> setGroupParams(SignalGroupId id,
                                               typename Group::Params p);

    void link(SignalGroupId id);
    void unlink(SignalGroupId id);
};