#pragma once

#include <road_network/demand/data/flow.hpp>
#include <road_network/demand/data/saturation_override.hpp>
#include <road_network/util/ids.hpp>
#include <unordered_set>

#include "bus/dirty_event_bus.hpp"
#include "bus/impl/dirty_events.hpp"
#include "bus/subscription_bag.hpp"

class Demand;
class NetworkEventBus;
class Graph;
class Transaction;

class DemandEditor {
   public:
    DemandEditor(Demand& demand, const Graph& graph, NetworkEventBus& bus,
                 DirtyEventBus& dirty);

    void setFlow(Transaction& tx, MovementId mov, NodeId at, Flow flow);
    void clearFlow(Transaction& tx, MovementId mov, NodeId at);
    void setSaturationOverride(Transaction& tx, MovementId mov, NodeId at,
                               SaturationOverride value);
    void clearSaturationOverride(Transaction& tx, MovementId mov, NodeId at);

    void registerSolve(Transaction& tx);
    void rederive();
    void flush();

   private:
    Demand& demand_;
    const Graph& graph_;
    NetworkEventBus& bus_;
    DirtyEventBus& dirty_;
    SubscriptionBag subs_;

    std::unordered_set<EntryId> reEntries_;

    void onMovementCreated(CreatedTx<MovementId, NodeId>);
    void onMovementRemoved(RemovedTx<MovementId, NodeId>);
    void onEntryRemoved(RemovedTx<EntryId, NodeId>);
};