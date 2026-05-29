#pragma once

#include "bus/network_event_bus.hpp"
#include "ui/coord_transform.hpp"

struct NetworkEventBus;
class NetworkScene;
class GraphData;
class GraphDerived;
class SignalData;

class NetworkScenePopulator {
   public:
    NetworkScenePopulator(NetworkScene&, NetworkEventBus&, const GraphData&,
                          const GraphDerived&, const SignalData&, CoordTransform);

   private:
    NetworkScene& scene_;
    const GraphData& data_;
    const GraphDerived& derived_;
    const SignalData& signalData_;
    CoordTransform xf_;

    EventBus<NodeCreated>::Subscription nodeCreated_;
    EventBus<NodeRederived>::Subscription nodeRederived_;
    EventBus<NodeRemoved>::Subscription nodeRemoved_;

    EventBus<EntryCreated>::Subscription entryCreated_;
    EventBus<EntryLanesChanged>::Subscription entryLanesChanged_;
    EventBus<EntryRemoved>::Subscription entryRemoved_;

    EventBus<ExitCreated>::Subscription exitCreated_;
    EventBus<ExitLanesChanged>::Subscription exitLanesChanged_;
    EventBus<ExitRemoved>::Subscription exitRemoved_;

    EventBus<CrosswalkSeriesCreated>::Subscription crosswalkSeriesCreated_;
    EventBus<CrosswalkSeriesUpdated>::Subscription crosswalkSeriesUpdated_;
    EventBus<CrosswalkSeriesRemoved>::Subscription crosswalkSeriesRemoved_;

    EventBus<CrosswalkCreated>::Subscription crosswalkCreated_;
    EventBus<CrosswalkUpdated>::Subscription crosswalkUpdated_;
    EventBus<CrosswalkRemoved>::Subscription crosswalkRemoved_;

    EventBus<MovementCreated>::Subscription movementCreated_;
    EventBus<MovementUpdated>::Subscription movementUpdated_;
    EventBus<MovementRemoved>::Subscription movementRemoved_;

    EventBus<SignalGroupCreated>::Subscription signalGroupCreated_;
    EventBus<SignalGroupRemoved>::Subscription signalGroupRemoved_;
};