#pragma once

#include "impl/event_bus.hpp"
#include "impl/network_events.hpp"
#include "impl/network_tags.hpp"

struct NetworkEventBus {
    EventBus<Created<NodeId>> nodeCreated;
    EventBus<Updated<NodeId>> nodeUpdated;
    EventBus<Updated<NodeId>> nodeUpdatedSignal;   // (rederived)
    EventBus<Updated<NodeId>> nodeUpdatedControl;  // (rederived)
    EventBus<Removed<NodeId>> nodeRemoved;
    EventBus<Created<EntryId, NodeId>> entryCreated;
    EventBus<Updated<EntryId, NodeId>> entryUpdated;
    EventBus<Removed<EntryId, NodeId>> entryRemoved;
    EventBus<Created<ExitId, NodeId>> exitCreated;
    EventBus<Updated<ExitId, NodeId>> exitUpdated;
    EventBus<Removed<ExitId, NodeId>> exitRemoved;
    EventBus<Created<CrosswalkSeriesId, NodeId>> crosswalkSeriesCreated;
    EventBus<Updated<CrosswalkSeriesId, NodeId>> crosswalkSeriesUpdated;
    EventBus<Removed<CrosswalkSeriesId, NodeId>> crosswalkSeriesRemoved;
    EventBus<Created<CrosswalkId, NodeId>> crosswalkCreated;
    EventBus<Updated<CrosswalkId, NodeId>> crosswalkUpdated;
    EventBus<Removed<CrosswalkId, NodeId>> crosswalkRemoved;
    EventBus<Created<MovementId, NodeId>> movementCreated;
    EventBus<Updated<MovementId, NodeId>> movementUpdated;
    EventBus<Removed<MovementId, NodeId>> movementRemoved;
    EventBus<Created<ClusterId, NodeId>> clusterCreated;
    EventBus<Removed<ClusterId, NodeId>> clusterRemoved;

    EventBus<Created<SignalGroupId, NodeId>> signalGroupCreated;
    EventBus<Updated<SignalGroupId, NodeId>> signalGroupUpdated;
    EventBus<Removed<SignalGroupId, NodeId>> signalGroupRemoved;
    EventBus<Created<TimingId, NodeId>> timingCreated;
    EventBus<Updated<TimingId, NodeId>> timingUpdated;
    EventBus<Removed<TimingId, NodeId>> timingRemoved;
    EventBus<Created<StageId, NodeId>> stageCreated;
    EventBus<Updated<StageId, NodeId>> stageUpdated;
    EventBus<Removed<StageId, NodeId>> stageRemoved;
    EventBus<Created<StageTransitionId, NodeId>> stageTransitionCreated;
    EventBus<Updated<StageTransitionId, NodeId>> stageTransitionUpdated;
    EventBus<Removed<StageTransitionId, NodeId>> stageTransitionRemoved;

    EventBus<Set<FlowTag, MovementId>> flowSet;
    EventBus<Cleared<FlowTag, MovementId>> flowCleared;
    EventBus<Set<SaturationOverrideTag, MovementId>> saturationOverrideSet;
    EventBus<Cleared<SaturationOverrideTag, MovementId>>
        saturationOverrideCleared;
    EventBus<Set<StreamPermittedOverrideTag, NodeId>>
        streamPermittedOverrideSet;
    EventBus<Cleared<StreamPermittedOverrideTag, NodeId>>
        streamPermittedOverrideCleared;
    EventBus<Set<StreamIntergreenOverrideTag, NodeId>>
        streamIntergreenOverrideSet;
    EventBus<Cleared<StreamIntergreenOverrideTag, NodeId>>
        streamIntergreenOverrideCleared;
};