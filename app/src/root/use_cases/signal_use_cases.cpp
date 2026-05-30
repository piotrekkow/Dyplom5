#include "signal_use_cases.hpp"

#include <QDebug>
#include <chrono>

#include "root/command/transaction.hpp"
#include "root/command_stack.hpp"
#include "root/editor/signal_editor.hpp"

SignalUseCases::SignalUseCases(SignalEditor& editor, CommandStack& cmdStack)
    : editor_(editor), cmdStack_(cmdStack) {}

SignalGroupId SignalUseCases::addMovementGroup(NodeId at,
                                               std::vector<ClusterId> clusters,
                                               bool isProtected,
                                               int intervalMin) {
    Transaction tx;
    auto id = editor_.addMovementGroupTx(tx, at, std::move(clusters),
                                         isProtected, intervalMin);
    cmdStack_.push(tx.commit());
    return id;
}

SignalGroupId SignalUseCases::addCrosswalkGroup(
    NodeId at, std::vector<CrosswalkId> crosswalks, int intervalMin) {
    Transaction tx;
    auto id =
        editor_.addCrosswalkGroupTx(tx, at, std::move(crosswalks), intervalMin);
    cmdStack_.push(tx.commit());
    return id;
}

SignalGroupId SignalUseCases::addTurnArrowGroup(NodeId at, MovementId movement,
                                                int intervalMin) {
    Transaction tx;
    auto id = editor_.addTurnArrowGroupTx(tx, at, movement, intervalMin);
    cmdStack_.push(tx.commit());
    return id;
}

void SignalUseCases::setIntervalMin(SignalGroupId id, int min) {
    Transaction tx;
    editor_.setIntervalMinTx(tx, id, min);
    cmdStack_.push(tx.commit());
}

void SignalUseCases::setProtected(SignalGroupId id, bool isProtected) {
    Transaction tx;
    editor_.setProtectedTx(tx, id, isProtected);
    cmdStack_.push(tx.commit());
}

void SignalUseCases::removeSignalGroup(SignalGroupId id) {
    Transaction tx;
    editor_.removeSignalGroupTx(tx, id);
    cmdStack_.push(tx.commit());
}

TimingId SignalUseCases::createTiming(NodeId at, int cycleLength) {
    Transaction tx;
    auto id = editor_.createTimingTx(tx, at, cycleLength);
    cmdStack_.push(tx.commit());
    return id;
}

void SignalUseCases::removeTiming(TimingId id) {
    Transaction tx;
    editor_.removeTimingTx(tx, id);
    cmdStack_.push(tx.commit());
}

void SignalUseCases::setAlwaysActive(TimingId timingId, SignalGroupId sgId) {
    Transaction tx;
    editor_.setAlwaysActiveTx(tx, timingId, sgId);
    cmdStack_.push(tx.commit());
}

std::expected<void, TimingError> SignalUseCases::setIntervals(
    TimingId timingId, SignalGroupId sgId, std::vector<Interval> intervals) {
    Transaction tx;
    auto res = editor_.setIntervalsTx(tx, timingId, sgId, std::move(intervals));
    if (!res) return res;
    cmdStack_.push(tx.commit());
    return {};
}

std::expected<void, TimingWarning> SignalUseCases::setMinIntervalLength(
    TimingId timingId, SignalGroupId sgId, int min) {
    Transaction tx;
    auto res = editor_.setMinIntervalLengthTx(tx, timingId, sgId, min);
    if (!res) return res;
    cmdStack_.push(tx.commit());
    return {};
}

// for debug print purposes
namespace {
static QString formatDuration(std::chrono::steady_clock::duration duration) {
    using namespace std::chrono;
    auto us = duration_cast<microseconds>(duration).count();
    if (us < 1000) return QString("%1 us").arg(us);
    auto ms = duration_cast<milliseconds>(duration).count();
    if (ms < 1000) return QString("%1 ms").arg(ms);
    double s = us / 1'000'000.0;
    return QString("%1 s").arg(s, 0, 'f', 3);
}
}  // namespace

std::optional<TimingId> SignalUseCases::runOptimizer(NodeId at,
                                                     int cycleLength) {
    Transaction tx;
    auto start = std::chrono::steady_clock::now();
    auto result = editor_.runOptimizerTx(tx, at, cycleLength);

    auto end = std::chrono::steady_clock::now();
    QString status =
        result ? "Successfully found solution." : "Found no solution.";
    qDebug().nospace().noquote()
        << "Optimizer finished. Time elapsed " << formatDuration(end - start)
        << ". " << status << '\n';

    if (result) cmdStack_.push(tx.commit());
    return result;
}
