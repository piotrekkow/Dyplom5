#include "control_editor.hpp"

#include <road_network/signal.hpp>
#include <variant>

#include "bus/network_event_bus.hpp"
#include "bus/network_events.hpp"
#include "root/command/transaction.hpp"

ControlEditor::ControlEditor(Control& control, const Signal& signal,
                             NetworkEventBus& bus)
    : control_(control), signal_(signal), bus_(bus) {}

//
//  TIMING LIFECYCLE TX
//

TimingId ControlEditor::createTimingTx(Transaction& tx, SignalConfigId at,
                                       int cycleLength) {
    auto mut = control_.createTiming(at, cycleLength);
    TimingId id = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.timingRemoved,
                  TimingRemoved{id, at});
    tx.recordEvent(bus_.timingCreated, TimingCreated{id, at});

    if (const auto* cfg = signal_.data().configs().get(at)) {
        for (SignalGroupId sgId : cfg->signalGroups()) {
            int minIntervalLength =
                std::visit([](const auto& x) { return x.intervalMin(); },
                           *signal_.data().signalGroups().get(sgId));
            createSequenceTx(tx, id, sgId, minIntervalLength);
        }
    }

    return id;
}

void ControlEditor::removeTimingTx(Transaction& tx, TimingId id) {
    SignalConfigId at = control_.data().timings().get(id)->at();
    auto mut = control_.removeTiming(id);
    tx.recordUndo(std::move(mut.undo), bus_.timingCreated,
                  TimingCreated{id, at});
    tx.recordEvent(bus_.timingRemoved, TimingRemoved{id, at});
}

//
//  SEQUENCE TX
//

void ControlEditor::setAlwaysActiveTx(Transaction& tx, TimingId timingId,
                                      SignalGroupId sgId) {
    auto mut = control_.setAlwaysActive(timingId, sgId);
    tx.recordUndo(std::move(mut.undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, mut.at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, mut.at});
}

std::expected<void, TimingError> ControlEditor::setIntervalsTx(
    Transaction& tx, TimingId timingId, SignalGroupId sgId,
    std::vector<Interval> intervals) {
    auto res = control_.setIntervals(timingId, sgId, std::move(intervals));
    if (!res) return std::unexpected(res.error());

    tx.recordUndo(std::move(res->undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, res->at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, res->at});
    return {};
}

std::expected<void, TimingWarning> ControlEditor::setMinIntervalLengthTx(
    Transaction& tx, TimingId timingId, SignalGroupId sgId, int min) {
    auto res = control_.setMinIntervalLength(timingId, sgId, min);
    if (!res) return std::unexpected(res.error());

    tx.recordUndo(std::move(res->undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, res->at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, res->at});
    return {};
}

//
//  EDITOR CONTEXT HOOKS
//

void ControlEditor::onConfigRemovedTx(Transaction& tx, SignalConfigId id) {
    const auto* cc = control_.data().configControls().get(id);
    if (!cc) return;

    auto timingsCopy = cc->timings;
    for (TimingId timingId : timingsCopy) removeTimingTx(tx, timingId);
}

void ControlEditor::onSignalGroupCreatedTx(Transaction& tx,
                                           SignalConfigId config,
                                           SignalGroupId sgId,
                                           int minIntervalLength) {
    const auto* cc = control_.data().configControls().get(config);
    if (!cc) return;

    for (TimingId timingId : cc->timings)
        createSequenceTx(tx, timingId, sgId, minIntervalLength);
}

void ControlEditor::onSignalGroupRemovedTx(Transaction& tx,
                                           SignalConfigId config,
                                           SignalGroupId sgId) {
    const auto* cc = control_.data().configControls().get(config);
    if (!cc) return;

    for (TimingId timingId : cc->timings) {
        if (control_.data().timings().get(timingId)->hasSequence(sgId))
            removeSequenceTx(tx, timingId, sgId);
    }
}

//
//  PRIVATE
//

void ControlEditor::createSequenceTx(Transaction& tx, TimingId timingId,
                                     SignalGroupId sgId,
                                     int minIntervalLength) {
    auto mut = control_.createSequence(timingId, sgId, minIntervalLength);
    tx.recordUndo(std::move(mut.undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, mut.at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, mut.at});
}

void ControlEditor::removeSequenceTx(Transaction& tx, TimingId timingId,
                                     SignalGroupId sgId) {
    auto mut = control_.removeSequence(timingId, sgId);
    tx.recordUndo(std::move(mut.undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, mut.at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, mut.at});
}
