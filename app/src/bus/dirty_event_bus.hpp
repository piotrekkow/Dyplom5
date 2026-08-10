#pragma once

#include "impl/dirty_events.hpp"
#include "impl/event_bus.hpp"
#include "road_network/util/ids.hpp"

struct DirtyEventBus {
    EventBus<UpdatedTx<NodeId>> nodeUpdated;
    EventBus<RemovedTx<NodeId>> nodeRemoved;
    EventBus<CreatedTx<MovementId, NodeId>> movementCreated;
    EventBus<RemovedTx<MovementId, NodeId>> movementRemoved;
    EventBus<RemovedTx<EntryId, NodeId>> entryRemoved;
    EventBus<UpdatedTx<CrosswalkId, NodeId>> crosswalkUpdated;
    EventBus<RemovedTx<CrosswalkId, NodeId>> crosswalkRemoved;
    EventBus<CreatedTx<SignalGroupId, NodeId>> signalGroupCreated;
    EventBus<RemovedTx<SignalGroupId, NodeId>> signalGroupRemoved;
};