#pragma once

#include <expected>
#include <road_network/control/control_errors.hpp>
#include <road_network/control/data/interval.hpp>
#include <road_network/util/ids.hpp>
#include <vector>

class ControlEditor;
class CommandStack;

class ControlUseCases {
   public:
    ControlUseCases(ControlEditor& editor, CommandStack& cmdStack);

    TimingId createTiming(SignalConfigId at, int cycleLength);
    void removeTiming(TimingId id);

    void setAlwaysActive(TimingId timingId, SignalGroupId sgId);
    std::expected<void, TimingError> setIntervals(
        TimingId timingId, SignalGroupId sgId, std::vector<Interval> intervals);
    std::expected<void, TimingWarning> setMinIntervalLength(TimingId timingId,
                                                            SignalGroupId sgId,
                                                            int min);

   private:
    ControlEditor& editor_;
    CommandStack& cmdStack_;
};
