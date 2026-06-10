#pragma once

#include <expected>
#include <vector>

#include "control/control_data.hpp"
#include "control/control_errors.hpp"
#include "control/data/interval.hpp"
#include "util/ids.hpp"
#include <functional>

class Control {
   public:
    const ControlData& data() const { return data_; }

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
    TimingMutation createSequence(TimingId id, SignalGroupId sg,
                                  int minIntervalLength);
    TimingMutation removeSequence(TimingId id, SignalGroupId sg);
    TimingMutation setAlwaysActive(TimingId id, SignalGroupId sg);
    std::expected<TimingMutation, TimingError> setIntervals(
        TimingId id, SignalGroupId sg, std::vector<Interval> intervals);
    std::expected<TimingMutation, TimingWarning> setMinIntervalLength(
        TimingId id, SignalGroupId sg, int min);

   private:
    ControlData data_;

    void createTimingImpl(SlotMap<TimingTag, Timing>::VacatedSlot vs);
    SlotMap<TimingTag, Timing>::VacatedSlot removeTimingImpl(TimingId id);
};
