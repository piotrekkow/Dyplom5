#pragma once

#include <expected>
#include <road_network/control.hpp>
#include <road_network/util/ids.hpp>
#include <vector>

class Graph;
class Signal;
class NetworkEventBus;
class Transaction;

class ControlEditor {
   public:
    ControlEditor(Control& control, const Signal& signal, const Graph& graph,
                  NetworkEventBus& bus);

    // Timing lifecycle Tx
    TimingId createTimingTx(Transaction& tx, SignalConfigId at,
                            int cycleLength);
    void removeTimingTx(Transaction& tx, TimingId id);

    // Sequence Tx
    void setAlwaysActiveTx(Transaction& tx, TimingId timingId,
                           SignalGroupId sgId);
    std::expected<void, TimingError> setIntervalsTx(
        Transaction& tx, TimingId timingId, SignalGroupId sgId,
        std::vector<Interval> intervals);

    // Tx hooks for EditorContext cross-domain cascades
    void onConfigRemovedTx(Transaction& tx, SignalConfigId id);
    void onSignalGroupCreatedTx(Transaction& tx, SignalConfigId config,
                                SignalGroupId sgId);
    void onSignalGroupRemovedTx(Transaction& tx, SignalConfigId config,
                                SignalGroupId sgId);
    // signal-domain parameters of the config changed (group intervalMin,
    // intergreen matrix, covered streams) — re-run the validation pass and
    // notify validity/completeness listeners
    void onConfigParamsChangedTx(Transaction& tx, SignalConfigId id);

   private:
    Control& control_;
    const Signal& signal_;
    const Graph& graph_;
    NetworkEventBus& bus_;

    void createSequenceTx(Transaction& tx, TimingId timingId,
                          SignalGroupId sgId);
    void removeSequenceTx(Transaction& tx, TimingId timingId,
                          SignalGroupId sgId);
    // schedules the deferred validation pass and publishes
    // TimingValidityChanged / ConfigCompletenessChanged for the config
    void onControlStale(Transaction& tx, SignalConfigId id);
};
