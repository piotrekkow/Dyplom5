#include "control_use_cases.hpp"

#include "root/command/transaction.hpp"
#include "root/command_stack.hpp"
#include "root/editor/control_editor.hpp"

ControlUseCases::ControlUseCases(ControlEditor& editor, CommandStack& cmdStack)
    : editor_(editor), cmdStack_(cmdStack) {}

TimingId ControlUseCases::createTiming(SignalConfigId at, int cycleLength) {
    Transaction tx;
    auto id = editor_.createTimingTx(tx, at, cycleLength);
    cmdStack_.push(tx.commit());
    return id;
}

void ControlUseCases::removeTiming(TimingId id) {
    Transaction tx;
    editor_.removeTimingTx(tx, id);
    cmdStack_.push(tx.commit());
}

void ControlUseCases::setAlwaysActive(TimingId timingId, SignalGroupId sgId) {
    Transaction tx;
    editor_.setAlwaysActiveTx(tx, timingId, sgId);
    cmdStack_.push(tx.commit());
}

std::expected<void, TimingError> ControlUseCases::setIntervals(
    TimingId timingId, SignalGroupId sgId, std::vector<Interval> intervals) {
    Transaction tx;
    auto res = editor_.setIntervalsTx(tx, timingId, sgId, std::move(intervals));
    if (!res) return res;
    cmdStack_.push(tx.commit());
    return {};
}
