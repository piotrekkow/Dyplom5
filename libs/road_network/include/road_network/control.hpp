#pragma once

#include <expected>
#include <unordered_set>
#include <vector>

#include "control/control_data.hpp"
#include "control/control_derived.hpp"
#include "control/control_errors.hpp"
#include "control/data/interval.hpp"
#include "util/ids.hpp"
#include <functional>

class GraphData;
class SignalData;
class SignalDerived;

class Control {
   public:
    const ControlData& data() const { return data_; }
    const ControlDerived& derived() const { return derived_; }

    struct ConfigMutation {
        SignalConfigId id;
        std::move_only_function<void()> undo;
    };

    template <typename EntityId>
    struct EntityMutation {
        EntityId id;
        SignalConfigId at;
        std::move_only_function<void()> undo;
    };

    struct TimingMutation {
        TimingId timingId;
        SignalGroupId sgId;
        SignalConfigId at;
        std::move_only_function<void()> undo;
    };

    // timing lifecycle
    EntityMutation<TimingId> createTiming(SignalConfigId at, int cycleLength);
    ConfigMutation removeTiming(TimingId id);

    // timing sequence mutations
    TimingMutation createSequence(TimingId id, SignalGroupId sg);
    TimingMutation removeSequence(TimingId id, SignalGroupId sg);
    TimingMutation setAlwaysActive(TimingId id, SignalGroupId sg);
    std::expected<TimingMutation, TimingError> setIntervals(
        TimingId id, SignalGroupId sg, std::vector<Interval> intervals);

    // Validation pass: re-checks timing validity and config completeness of
    // dirty configs against the current graph/signal state. Mutations mark
    // their config dirty; external parameter changes (group intervalMin,
    // intergreen matrix, cluster/crosswalk structure) are flagged via
    // markStale by the coordinating layer.
    void markStale(SignalConfigId id) { dirty_.insert(id); }
    void solve(const GraphData& graph, const SignalData& signal,
               const SignalDerived& signalDerived);

   private:
    ControlData data_;
    ControlDerived derived_;

    std::unordered_set<SignalConfigId> dirty_;
    void recomputeDerived(const GraphData& graph, const SignalData& signal,
                          const SignalDerived& signalDerived,
                          SignalConfigId id);

    void createTimingImpl(SlotMap<TimingTag, Timing>::VacatedSlot vs);
    SlotMap<TimingTag, Timing>::VacatedSlot removeTimingImpl(TimingId id);
};
