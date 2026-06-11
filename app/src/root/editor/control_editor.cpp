#include "control_editor.hpp"

#include <road_network/graph.hpp>
#include <road_network/signal.hpp>

#include "bus/network_event_bus.hpp"
#include "bus/network_events.hpp"
#include "root/command/solve_priority.hpp"
#include "root/command/transaction.hpp"

ControlEditor::ControlEditor(Control& control, const Signal& signal,
                             const Graph& graph, NetworkEventBus& bus)
    : control_(control), signal_(signal), graph_(graph), bus_(bus) {}

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
        for (SignalGroupId sgId : cfg->signalGroups())
            createSequenceTx(tx, id, sgId);
    }

    onControlStale(tx, at);
    return id;
}

void ControlEditor::removeTimingTx(Transaction& tx, TimingId id) {
    SignalConfigId at = control_.data().timings().get(id)->at();
    auto mut = control_.removeTiming(id);
    tx.recordUndo(std::move(mut.undo), bus_.timingCreated,
                  TimingCreated{id, at});
    tx.recordEvent(bus_.timingRemoved, TimingRemoved{id, at});
    onControlStale(tx, at);
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
    onControlStale(tx, mut.at);
}

std::expected<void, TimingError> ControlEditor::setIntervalsTx(
    Transaction& tx, TimingId timingId, SignalGroupId sgId,
    std::vector<Interval> intervals) {
    auto res = control_.setIntervals(timingId, sgId, std::move(intervals));
    if (!res) return std::unexpected(res.error());

    SignalConfigId at = res->at;
    tx.recordUndo(std::move(res->undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, at});
    tx.recordEvent(bus_.sequenceChanged, SequenceChanged{timingId, sgId, at});
    onControlStale(tx, at);
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
                                           SignalGroupId sgId) {
    if (const auto* cc = control_.data().configControls().get(config)) {
        for (TimingId timingId : cc->timings)
            createSequenceTx(tx, timingId, sgId);
    }
    onControlStale(tx, config);
}

void ControlEditor::onSignalGroupRemovedTx(Transaction& tx,
                                           SignalConfigId config,
                                           SignalGroupId sgId) {
    if (const auto* cc = control_.data().configControls().get(config)) {
        for (TimingId timingId : cc->timings) {
            if (control_.data().timings().get(timingId)->hasSequence(sgId))
                removeSequenceTx(tx, timingId, sgId);
        }
    }
    onControlStale(tx, config);
}

void ControlEditor::onConfigParamsChangedTx(Transaction& tx,
                                            SignalConfigId id) {
    control_.markStale(id);
    tx.recordUndoSilent([this, id] { control_.markStale(id); });
    onControlStale(tx, id);
}

//
//  PRIVATE
//

void ControlEditor::createSequenceTx(Transaction& tx, TimingId timingId,
                                     SignalGroupId sgId) {
    auto mut = control_.createSequence(timingId, sgId);
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

void ControlEditor::onControlStale(Transaction& tx, SignalConfigId id) {
    const auto* cfg = signal_.data().configs().get(id);
    if (!cfg) return;  // config already removed — nothing left to re-validate
    IntersectionId at = cfg->at();
    tx.recordEvent(bus_.configCompletenessChanged,
                   ConfigCompletenessChanged{id, at});
    tx.recordUndoEvent(bus_.configCompletenessChanged,
                       ConfigCompletenessChanged{id, at});

    if (const auto* cc = control_.data().configControls().get(id)) {
        for (TimingId tId : cc->timings) {
            tx.recordEvent(bus_.timingValidityChanged,
                           TimingValidityChanged{tId, id});
            tx.recordUndoEvent(bus_.timingValidityChanged,
                               TimingValidityChanged{tId, id});
        }
    }

    tx.addSolve(SolvePriority::CONTROL, [this] {
        control_.solve(graph_.data(), signal_.data(), signal_.derived());
    });
}
