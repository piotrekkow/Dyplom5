#include "demand_editor.hpp"

#include <road_network/demand.hpp>

#include "bus/network_event_bus.hpp"
#include "bus/network_events.hpp"
#include "root/command/solve_priority.hpp"
#include "root/command/transaction.hpp"

void DemandEditor::onMovementCreatedTx(Transaction& tx, MovementId id) {
    setFlowTx(tx, id, {0.f});
    setSaturationOverrideTx(tx, id, {});
}

void DemandEditor::onMovementRemovedTx(Transaction& tx, MovementId id) {
    eraseFlowTx(tx, id);
    eraseSaturationOverrideTx(tx, id);
}

void DemandEditor::setFlowTx(Transaction& tx, MovementId at, Flow flow) {
    auto mut = demand_.setFlow(at, flow);
    tx.recordUndo(std::move(mut.undo), bus_.flowChanged, FlowChanged{at});
    tx.recordEvent(bus_.flowChanged, FlowChanged{at});
    tx.addSolve(SolvePriority::DEMAND, [this] { demand_.solve(graph_); });
}

void DemandEditor::eraseFlowTx(Transaction& tx, MovementId at) {
    auto mut = demand_.eraseFlow(at);
    tx.recordUndo(std::move(mut.undo), bus_.flowChanged, FlowChanged{at});
    tx.recordEvent(bus_.flowErased, FlowErased{at});
    tx.addSolve(SolvePriority::DEMAND, [this] { demand_.solve(graph_); });
}

void DemandEditor::setSaturationOverrideTx(Transaction& tx, MovementId at,
                                           SaturationOverride override) {
    auto mut = demand_.setSaturationOverride(at, override);
    tx.recordUndo(std::move(mut.undo), bus_.saturationOverrideChanged,
                  SaturationOverrideChanged{at});
    tx.recordEvent(bus_.saturationOverrideChanged,
                   SaturationOverrideChanged{at});
    tx.addSolve(SolvePriority::DEMAND, [this] { demand_.solve(graph_); });
}

void DemandEditor::eraseSaturationOverrideTx(Transaction& tx, MovementId at) {
    auto mut = demand_.eraseSaturationOverride(at);
    tx.recordUndo(std::move(mut.undo), bus_.saturationOverrideChanged,
                  SaturationOverrideChanged{at});
    tx.recordEvent(bus_.saturationOverrideErased, SaturationOverrideErased{at});
    tx.addSolve(SolvePriority::DEMAND, [this] { demand_.solve(graph_); });
}
