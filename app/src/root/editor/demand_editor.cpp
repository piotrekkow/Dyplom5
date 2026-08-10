#include "demand_editor.hpp"

#include <road_network/demand.hpp>
#include <road_network/graph.hpp>

#include "bus/network_event_bus.hpp"
#include "util/apply_tx.hpp"

DemandEditor::DemandEditor(Demand& demand, const Graph& graph,
                           NetworkEventBus& bus, DirtyEventBus& dirty)
    : demand_(demand), graph_(graph), bus_(bus), dirty_(dirty) {
    subs_.add(dirty_.movementCreated, [this](CreatedTx<MovementId, NodeId> e) {
        onMovementCreated(e);
    });
    subs_.add(dirty_.movementRemoved, [this](RemovedTx<MovementId, NodeId> e) {
        onMovementRemoved(e);
    });
    subs_.add(dirty_.entryRemoved,
              [this](RemovedTx<EntryId, NodeId> e) { onEntryRemoved(e); });
}

void DemandEditor::onMovementCreated(CreatedTx<MovementId, NodeId> e) {
    setFlow(e.tx, e.id, e.at, {0});
    setSaturationOverride(e.tx, e.id, e.at, {});
}

void DemandEditor::onMovementRemoved(RemovedTx<MovementId, NodeId> e) {
    clearFlow(e.tx, e.id, e.at);
    clearSaturationOverride(e.tx, e.id, e.at);
}

void DemandEditor::onEntryRemoved(RemovedTx<EntryId, NodeId> e) {
    demand_.markDirty(e.id);
    demand_.markDirty(e.at);
}

void DemandEditor::setFlow(Transaction& tx, MovementId mov, NodeId at,
                           Flow flow) {
    auto mut = demand_.setFlow(mov, at, flow);
    applyTxUpdate(tx, std::move(mut), bus_.flowSet,
                  Set<FlowTag, MovementId>{.at = mov});
}

void DemandEditor::clearFlow(Transaction& tx, MovementId mov, NodeId at) {
    auto mut = demand_.clearFlow(mov, at);
    applyTxUpdate(tx, std::move(mut), bus_.flowCleared,
                  Cleared<FlowTag, MovementId>{.at = mov});
}

void DemandEditor::setSaturationOverride(Transaction& tx, MovementId mov,
                                         NodeId at, SaturationOverride value) {
    auto mut = demand_.setSaturationOverride(mov, at, value);
    applyTxUpdate(tx, std::move(mut), bus_.saturationOverrideSet,
                  Set<SaturationOverrideTag, MovementId>{.at = mov});
}

void DemandEditor::clearSaturationOverride(Transaction& tx, MovementId mov,
                                           NodeId at) {
    auto mut = demand_.clearSaturationOverride(mov, at);
    applyTxUpdate(tx, std::move(mut), bus_.saturationOverrideCleared,
                  Cleared<SaturationOverrideTag, MovementId>{.at = mov});
}

void DemandEditor::registerSolve(Transaction& tx) {
    reEntries_.clear();
    tx.addSolve(SolvePriority::DEMAND, {.rederive = [this] { rederive(); },
                                        .flush = [this] { flush(); }});
}

void DemandEditor::rederive() {
    auto r = demand_.solve(graph_.data());
    reEntries_.insert(r.entries.begin(), r.entries.end());
}

void DemandEditor::flush() {
    for (auto id : reEntries_) {
        if (graph_.data().entries().alive(id)) {
            const auto at = graph_.parentOf(id);
            bus_.entryUpdated.publish(
                Updated<EntryId, NodeId>{.id = id, .at = at});
        }
    }
    reEntries_.clear();
}