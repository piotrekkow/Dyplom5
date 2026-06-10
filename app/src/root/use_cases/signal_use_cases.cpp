#include "signal_use_cases.hpp"

#include <QDebug>
#include <chrono>

#include "root/command/transaction.hpp"
#include "root/command_stack.hpp"
#include "root/editor/signal_editor.hpp"

SignalUseCases::SignalUseCases(SignalEditor& editor, EditorContext& ec,
                               CommandStack& cmdStack)
    : editor_(editor), ec_(ec), cmdStack_(cmdStack) {}

SignalConfigId SignalUseCases::createConfig(NodeId at, std::string name) {
    Transaction tx;
    auto id = editor_.createConfigTx(tx, at, std::move(name));
    cmdStack_.push(tx.commit());
    return id;
}

void SignalUseCases::renameConfig(SignalConfigId id, std::string name) {
    Transaction tx;
    editor_.renameConfigTx(tx, id, std::move(name));
    cmdStack_.push(tx.commit());
}

void SignalUseCases::removeConfig(SignalConfigId id) {
    Transaction tx;
    ec_.removeSignalConfigTx(tx, id);
    cmdStack_.push(tx.commit());
}

SignalGroupId SignalUseCases::addMovementGroup(SignalConfigId config,
                                               std::vector<ClusterId> clusters,
                                               bool isProtected,
                                               int intervalMin) {
    Transaction tx;
    auto id = ec_.addMovementGroupTx(tx, config, std::move(clusters),
                                     isProtected, intervalMin);
    cmdStack_.push(tx.commit());
    return id;
}

SignalGroupId SignalUseCases::addCrosswalkGroup(
    SignalConfigId config, std::vector<CrosswalkId> crosswalks,
    int intervalMin) {
    Transaction tx;
    auto id = ec_.addCrosswalkGroupTx(tx, config, std::move(crosswalks),
                                      intervalMin);
    cmdStack_.push(tx.commit());
    return id;
}

SignalGroupId SignalUseCases::addTurnArrowGroup(SignalConfigId config,
                                                MovementId movement,
                                                int intervalMin) {
    Transaction tx;
    auto id = ec_.addTurnArrowGroupTx(tx, config, movement, intervalMin);
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
    ec_.removeSignalGroupTx(tx, id);
    cmdStack_.push(tx.commit());
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

std::optional<EditorContext::OptimizerRunResult> SignalUseCases::runOptimizer(
    NodeId at, int cycleLength) {
    Transaction tx;
    auto start = std::chrono::steady_clock::now();
    auto result = ec_.runOptimizerTx(tx, at, cycleLength);

    auto end = std::chrono::steady_clock::now();
    QString status =
        result ? "Successfully found solution." : "Found no solution.";
    qDebug().nospace().noquote()
        << "Optimizer finished. Time elapsed " << formatDuration(end - start)
        << ". " << status << '\n';

    if (result) cmdStack_.push(tx.commit());
    return result;
}
