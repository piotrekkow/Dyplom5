#pragma once

#include <expected>
#include <optional>
#include <road_network/signal/data/interval.hpp>
#include <road_network/signal/signal_errors.hpp>
#include <road_network/util/ids.hpp>
#include <signal_optimizer/optimizer.hpp>
#include <vector>

class SignalEditor;
class CommandStack;

class SignalUseCases {
   public:
    SignalUseCases(SignalEditor& editor, CommandStack& cmdStack);

    SignalGroupId addMovementGroup(NodeId at, std::vector<ClusterId> clusters,
                                   bool isProtected, int intervalMin);
    SignalGroupId addCrosswalkGroup(NodeId at,
                                    std::vector<CrosswalkId> crosswalks,
                                    int intervalMin);
    SignalGroupId addTurnArrowGroup(NodeId at, MovementId movement,
                                    int intervalMin);

    void setIntervalMin(SignalGroupId id, int min);
    void setProtected(SignalGroupId id, bool isProtected);
    void removeSignalGroup(SignalGroupId id);

    TimingId createTiming(NodeId at, int cycleLength);
    void removeTiming(TimingId id);

    void setAlwaysActive(TimingId timingId, SignalGroupId sgId);
    std::expected<void, TimingError> setIntervals(
        TimingId timingId, SignalGroupId sgId, std::vector<Interval> intervals);
    std::expected<void, TimingWarning> setMinIntervalLength(TimingId timingId,
                                                            SignalGroupId sgId,
                                                            int min);

    std::optional<TimingId> runOptimizer(NodeId at, int cycleLength);

   private:
    SignalEditor& editor_;
    CommandStack& cmdStack_;
};
