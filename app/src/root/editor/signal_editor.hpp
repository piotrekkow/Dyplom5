#pragma once

#include <expected>
#include <regulations/types/signal_kind.hpp>
#include <unordered_set>

#include "bus/dirty_event_bus.hpp"
#include "bus/impl/dirty_events.hpp"
#include "bus/subscription_bag.hpp"
#include "root/editor/util/editor_errors.hpp"

class Signal;
class NetworkEventBus;
class Graph;
class Transaction;
class DirtyEventBus;
class Regulations;

class SignalEditor {
   public:
    SignalEditor(Signal& signal, const Graph& graph, NetworkEventBus& bus,
                 DirtyEventBus& dirty, const Regulations& regulations);

    std::expected<SignalGroupId, SignalGroupErrors> createMovementGroup(
        Transaction& tx, NodeId node, std::vector<ClusterId> clusters);
    std::expected<SignalGroupId, SignalGroupErrors> createCrosswalkGroup(
        Transaction& tx, NodeId node, std::vector<CrosswalkId> crosswalks);

    std::expected<void, SignalGroupErrors> setSgProtection(Transaction& tx,
                                                           SignalGroupId id,
                                                           bool request);
    void setSgIntervalMinFixed(Transaction& tx, SignalGroupId id, int min);

    void removeSignalGroup(Transaction& tx, SignalGroupId id);

    void registerSolve(Transaction& tx);
    void rederive();
    void flush();

   private:
    Signal& signal_;
    const Graph& graph_;
    NetworkEventBus& bus_;
    DirtyEventBus& dirty_;
    const Regulations& regulations_;
    SubscriptionBag subs_;
    std::unordered_set<NodeId> reNodes_;

    void onNodeUpdated(UpdatedTx<NodeId>);
    void onNodeRemoved(RemovedTx<NodeId>);
    void onEntryRemoved(RemovedTx<EntryId, NodeId>);
    void onMovementRemoved(RemovedTx<MovementId, NodeId>);
    void onCrosswalkUpdated(UpdatedTx<CrosswalkId, NodeId>);
    void onCrosswalkRemoved(RemovedTx<CrosswalkId, NodeId>);
    void removeGroupsWithStream(Transaction& tx, StreamId stream, NodeId at);

    void setSgKind(Transaction& tx, SignalGroupId id, SignalKind kind);
};