#pragma once

#include <road_network/demand/data/flow.hpp>
#include <road_network/demand/data/saturation_override.hpp>
#include <road_network/util/ids.hpp>

class Transaction;
class Demand;
class NetworkEventBus;
class GraphData;

class DemandEditor {
   public:
    DemandEditor(Demand& demand, const GraphData& graph, NetworkEventBus& bus)
        : demand_(demand), graph_(graph), bus_(bus) {}

    void onMovementCreatedTx(Transaction& tx, MovementId id);
    void onMovementRemovedTx(Transaction& tx, MovementId id);

    void setFlowTx(Transaction& tx, MovementId at, Flow flow);
    void eraseFlowTx(Transaction& tx, MovementId at);
    void setSaturationOverrideTx(Transaction& tx, MovementId at,
                                 SaturationOverride override);
    void eraseSaturationOverrideTx(Transaction& tx, MovementId at);

   private:
    Demand& demand_;
    const GraphData& graph_;
    NetworkEventBus& bus_;
};
