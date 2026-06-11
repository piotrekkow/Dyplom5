#include "demand_editor.hpp"

#include <road_network/demand.hpp>
#include <road_network/graph/derived/graph_geometry.hpp>
#include <road_network/graph/graph_data.hpp>

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
    onDemandStale(tx, at);
}

void DemandEditor::eraseFlowTx(Transaction& tx, MovementId at) {
    auto mut = demand_.eraseFlow(at);
    tx.recordUndo(std::move(mut.undo), bus_.flowChanged, FlowChanged{at});
    tx.recordEvent(bus_.flowErased, FlowErased{at});
    onDemandStale(tx, at);
}

void DemandEditor::setSaturationOverrideTx(Transaction& tx, MovementId at,
                                           SaturationOverride override) {
    auto mut = demand_.setSaturationOverride(at, override);
    tx.recordUndo(std::move(mut.undo), bus_.saturationOverrideChanged,
                  SaturationOverrideChanged{at});
    tx.recordEvent(bus_.saturationOverrideChanged,
                   SaturationOverrideChanged{at});
    onDemandStale(tx, at);
}

void DemandEditor::eraseSaturationOverrideTx(Transaction& tx, MovementId at) {
    auto mut = demand_.eraseSaturationOverride(at);
    tx.recordUndo(std::move(mut.undo), bus_.saturationOverrideChanged,
                  SaturationOverrideChanged{at});
    tx.recordEvent(bus_.saturationOverrideErased, SaturationOverrideErased{at});
    onDemandStale(tx, at);
}

void DemandEditor::onDemandStale(Transaction& tx, MovementId at) {
    if (const auto* m = graph_.movements().get(at)) {
        EntryId entry = m->from();
        tx.recordEvent(bus_.demandRederived, DemandRederived{entry});
        tx.recordUndoEvent(bus_.demandRederived, DemandRederived{entry});
        if (auto opp = graph_geometry::findOpposingEntry(graph_, entry)) {
            tx.recordEvent(bus_.demandRederived, DemandRederived{*opp});
            tx.recordUndoEvent(bus_.demandRederived, DemandRederived{*opp});
        }
    }
    tx.addSolve(SolvePriority::DEMAND, [this] { demand_.solve(graph_); });
}
