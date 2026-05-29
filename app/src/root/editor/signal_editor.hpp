#pragma once

#include <expected>
#include <optional>
#include <road_network/signal.hpp>
#include <road_network/signal/data/interval.hpp>
#include <road_network/signal/signal_errors.hpp>
#include <road_network/util/ids.hpp>
#include <signal_optimizer/optimizer.hpp>
#include <vector>

class Graph;
class Demand;
class NetworkEventBus;
class Transaction;

class SignalEditor {
   public:
    SignalEditor(Signal& signal, const Graph& graph, const Demand& demand,
                 NetworkEventBus& bus);

    // Signal group Tx
    SignalGroupId addMovementGroupTx(Transaction& tx, NodeId at,
                                     std::vector<ClusterId> clusters,
                                     bool isProtected, int intervalMin);
    SignalGroupId addCrosswalkGroupTx(Transaction& tx, NodeId at,
                                      std::vector<CrosswalkId> crosswalks,
                                      int intervalMin);
    SignalGroupId addTurnArrowGroupTx(Transaction& tx, NodeId at,
                                      MovementId movement, int intervalMin);
    void setIntervalMinTx(Transaction& tx, SignalGroupId id, int min);
    void setProtectedTx(Transaction& tx, SignalGroupId id, bool isProtected);
    void removeSignalGroupTx(Transaction& tx, SignalGroupId id);

    // Timing lifecycle Tx
    TimingId createTimingTx(Transaction& tx, NodeId at, int cycleLength);
    void removeTimingTx(Transaction& tx, TimingId id);

    // Sequence Tx
    void setAlwaysActiveTx(Transaction& tx, TimingId timingId,
                           SignalGroupId sgId);
    std::expected<void, TimingError> setIntervalsTx(
        Transaction& tx, TimingId timingId, SignalGroupId sgId,
        std::vector<Interval> intervals);
    std::expected<void, TimingWarning> setMinIntervalLengthTx(
        Transaction& tx, TimingId timingId, SignalGroupId sgId, int min);

    // Optimizer Tx
    std::optional<signal_optimizer::OptimizeResult> runOptimizerTx(
        Transaction& tx, NodeId at, int cycleLength);

    // Tx variants for EditorContext cross-system cascade
    void removeNodeSignalsTx(Transaction& tx, NodeId id);
    void onEntryLayoutClearedTx(Transaction& tx, NodeId at,
                                 const std::vector<ClusterId>& clusters,
                                 const std::vector<MovementId>& movements);
    void onCrosswalkRemovedTx(Transaction& tx, CrosswalkId id, NodeId at);

   private:
    Signal& signal_;
    const Graph& graph_;
    const Demand& demand_;
    NetworkEventBus& bus_;

    SignalGroupId addSignalGroupTx(Transaction& tx,
                                    Signal::EntityMutation<SignalGroupId> mut);
    void createSequenceTx(Transaction& tx, TimingId timingId,
                           SignalGroupId sgId, int minIntervalLength,
                           NodeId at);
    void removeSequenceTx(Transaction& tx, TimingId timingId,
                           SignalGroupId sgId, NodeId at);
};
