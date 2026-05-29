#pragma once
#include "event_bus.hpp"
#include "network_events.hpp"

struct NetworkEventBus {
    EventBus<NodeCreated> nodeCreated;
    EventBus<NodeRemoved> nodeRemoved;

    EventBus<NodeRederived> nodeRederived;

    EventBus<EntryCreated> entryCreated;
    EventBus<EntryRemoved> entryRemoved;

    EventBus<ExitCreated> exitCreated;
    EventBus<ExitRemoved> exitRemoved;

    EventBus<CrosswalkSeriesCreated> crosswalkSeriesCreated;
    EventBus<CrosswalkSeriesUpdated> crosswalkSeriesUpdated;
    EventBus<CrosswalkSeriesRemoved> crosswalkSeriesRemoved;
    EventBus<CrosswalkCreated> crosswalkCreated;
    EventBus<CrosswalkUpdated> crosswalkUpdated;
    EventBus<CrosswalkRemoved> crosswalkRemoved;

    EventBus<MovementCreated> movementCreated;
    EventBus<MovementUpdated> movementUpdated;
    EventBus<MovementRemoved> movementRemoved;

    EventBus<EntryLanesChanged> entryLanesChanged;
    EventBus<ExitLanesChanged> exitLanesChanged;

    EventBus<ClusterCreated> clusterCreated;
    EventBus<ClusterRemoved> clusterRemoved;

    EventBus<FlowChanged> flowChanged;
    EventBus<FlowErased> flowErased;

    EventBus<SaturationOverrideChanged> saturationOverrideChanged;
    EventBus<SaturationOverrideErased> saturationOverrideErased;

    EventBus<StreamPermittedOverrideChanged> streamPermittedOverrideChanged;
    EventBus<StreamPermittedOverrideErased> streamPermittedOverrideErased;

    EventBus<StreamIntergreenOverrideChanged> streamIntergreenOverrideChanged;
    EventBus<StreamIntergreenOverrideErased> streamIntergreenOverrideErased;

    EventBus<SignalGroupCreated> signalGroupCreated;
    EventBus<SignalGroupRemoved> signalGroupRemoved;
    EventBus<SignalGroupChanged> signalGroupChanged;

    EventBus<TimingCreated> timingCreated;
    EventBus<TimingRemoved> timingRemoved;

    EventBus<SequenceChanged> sequenceChanged;
};