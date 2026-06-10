#pragma once

#include <expected>
#include <road_network/control.hpp>
#include <road_network/util/ids.hpp>
#include <vector>

class Signal;
class NetworkEventBus;
class Transaction;

class ControlEditor {
   public:
    ControlEditor(Control& control, const Signal& signal, NetworkEventBus& bus);

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
    std::expected<void, TimingWarning> setMinIntervalLengthTx(
        Transaction& tx, TimingId timingId, SignalGroupId sgId, int min);

    // Tx hooks for EditorContext cross-domain cascades
    void onConfigRemovedTx(Transaction& tx, SignalConfigId id);
    void onSignalGroupCreatedTx(Transaction& tx, SignalConfigId config,
                                SignalGroupId sgId, int minIntervalLength);
    void onSignalGroupRemovedTx(Transaction& tx, SignalConfigId config,
                                SignalGroupId sgId);

   private:
    Control& control_;
    const Signal& signal_;
    NetworkEventBus& bus_;

    void createSequenceTx(Transaction& tx, TimingId timingId,
                          SignalGroupId sgId, int minIntervalLength);
    void removeSequenceTx(Transaction& tx, TimingId timingId,
                          SignalGroupId sgId);
};
