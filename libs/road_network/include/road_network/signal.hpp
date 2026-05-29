#pragma once

#include <expected>
#include <unordered_set>
#include <vector>

#include "signal/data/interval.hpp"
#include "signal/signal_data.hpp"
#include "signal/signal_derived.hpp"
#include "signal/signal_errors.hpp"
#include "util/ids.hpp"
#include <functional>

class GraphDerived;

class Signal {
   public:
    const SignalData& data() const { return data_; }
    const SignalDerived& derived() const { return derived_; }

    struct NodeMutation {
        NodeId id;
        std::move_only_function<void()> undo;
    };

    template <typename EntityId>
    struct EntityMutation {
        EntityId id;
        NodeId at;
        std::move_only_function<void()> undo;
    };

    struct TimingMutation {
        TimingId timingId;
        SignalGroupId sgId;
        NodeId at;
        std::move_only_function<void()> undo;
    };

    // signal group creation — specific type constructed in editor
    EntityMutation<SignalGroupId> createSignalGroup(SignalGroup sg);
    EntityMutation<SignalGroupId> createMovementGroup(
        NodeId at, std::vector<ClusterId> clusters,
        std::vector<MovementId> movements, bool isProtected, int intervalMin);
    EntityMutation<SignalGroupId> createCrosswalkGroup(
        NodeId at, std::vector<CrosswalkId> crosswalks, int intervalMin);
    EntityMutation<SignalGroupId> createTurnArrowGroup(NodeId at,
                                                       MovementId movement,
                                                       int intervalMin);

    // signal group mutations
    EntityMutation<SignalGroupId> setIntervalMin(SignalGroupId id, int min);
    EntityMutation<SignalGroupId> setProtected(SignalGroupId id,
                                               bool isProtected);
    NodeMutation removeSignalGroup(SignalGroupId id);

    // timing lifecycle
    EntityMutation<TimingId> createTiming(NodeId at, int cycleLength);
    NodeMutation removeTiming(TimingId id);

    // timing sequence mutations
    TimingMutation createSequence(TimingId id, SignalGroupId sg,
                                  int minIntervalLength);
    TimingMutation removeSequence(TimingId id, SignalGroupId sg);
    TimingMutation setAlwaysActive(TimingId id, SignalGroupId sg);
    std::expected<TimingMutation, TimingError> setIntervals(
        TimingId id, SignalGroupId sg, std::vector<Interval> intervals);
    std::expected<TimingMutation, TimingWarning> setMinIntervalLength(
        TimingId id, SignalGroupId sg, int min);

    void solve(const GraphDerived& graphDerived) {
        for (NodeId id : dirty_) recomputeDerived(graphDerived, id);
        dirty_.clear();
    }

   private:
    SignalData data_;
    SignalDerived derived_;

    std::unordered_set<NodeId> dirty_;
    void recomputeDerived(const GraphDerived& graphDerived, NodeId id);

    void createSignalGroupImpl(
        SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot vs);
    SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot removeSignalGroupImpl(
        SignalGroupId id);
    void createTimingImpl(SlotMap<TimingTag, Timing>::VacatedSlot vs);
    SlotMap<TimingTag, Timing>::VacatedSlot removeTimingImpl(TimingId id);
};