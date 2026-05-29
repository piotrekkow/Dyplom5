#include "editor_context.hpp"

#include <expected>

#include "bus/network_event_bus.hpp"
#include "command/transaction.hpp"
#include "document.hpp"
#include "road_network/util/ids.hpp"
#include "use_cases/demand_use_cases.hpp"
#include "use_cases/graph_use_cases.hpp"
#include "use_cases/signal_use_cases.hpp"

EditorContext::EditorContext(Document& doc, NetworkEventBus& bus)
    : doc_(doc),
      cmdStack_(),
      graphEditor_(doc.graph(), bus),
      demandEditor_(doc.demand(), doc.graph().data(), bus),
      signalEditor_(doc.signal(), doc.graph(), doc.demand(), bus),
      graphUseCases_(
          std::make_unique<GraphUseCases>(graphEditor_, *this, cmdStack_)),
      demandUseCases_(
          std::make_unique<DemandUseCases>(demandEditor_, cmdStack_)),
      signalUseCases_(
          std::make_unique<SignalUseCases>(signalEditor_, cmdStack_)) {}

EditorContext::~EditorContext() = default;

std::expected<std::vector<MovementId>, MovementLayoutError>
EditorContext::setEntryLayoutTx(Transaction& tx, EntryId id,
                                std::vector<GraphEditor::MovementSpec> specs,
                                std::vector<bool> shared) {
    auto err = graphEditor_.validateLayout(id, specs, shared);
    if (!err) return std::unexpected(err.error());

    auto entry = doc_.graph().data().entries().get(id);
    assert(entry);

    for (auto m : entry->movements()) {
        demandEditor_.onMovementRemovedTx(tx, m);
    }
    return graphEditor_.setEntryLayoutTx(tx, id, std::move(specs),
                                         std::move(shared));
}

void EditorContext::removeCrosswalkTx(Transaction& tx, CrosswalkId id) {
    assert(doc_.graph().data().crosswalks().alive(id));
    auto* cw = doc_.graph().data().crosswalks().get(id);
    NodeId at = doc_.graph().data().crosswalkSeries().get(cw->parent())->at();
    signalEditor_.onCrosswalkRemovedTx(tx, id, at);
    graphEditor_.removeCrosswalkTx(tx, id);
}

void EditorContext::removeCrosswalkSeriesTx(Transaction& tx,
                                            CrosswalkSeriesId id) {
    assert(doc_.graph().data().crosswalkSeries().alive(id));
    auto* cs = doc_.graph().data().crosswalkSeries().get(id);
    NodeId at = cs->at();
    auto crosswalksCopy = cs->crosswalks();
    for (CrosswalkId cid : crosswalksCopy)
        signalEditor_.onCrosswalkRemovedTx(tx, cid, at);
    graphEditor_.removeCrosswalkSeriesTx(tx, id);
}

void EditorContext::removeEntryTx(Transaction& tx, EntryId id) {
    assert(doc_.graph().data().entries().alive(id));
    auto& data = doc_.graph().data();
    auto nodeId = data.entries().get(id)->at();
    removeEntryLayoutsTx(tx, nodeId);
    graphEditor_.removeEntryTx(tx, id);
}

void EditorContext::removeExitTx(Transaction& tx, ExitId id) {
    assert(doc_.graph().data().exits().alive(id));
    auto& data = doc_.graph().data();
    auto nodeId = data.exits().get(id)->at();
    removeEntryLayoutsTx(tx, nodeId);
    graphEditor_.removeExitTx(tx, id);
}

void EditorContext::setEntryLanesTx(Transaction& tx, EntryId id,
                                    std::vector<EntryLaneSpec> specs) {
    assert(doc_.graph().data().entries().alive(id));
    removeEntryLayoutTx(tx, id);
    graphEditor_.setEntryLanesTx(tx, id, std::move(specs));
}

void EditorContext::setExitLanesTx(Transaction& tx, ExitId id,
                                   std::vector<ExitLaneSpec> specs) {
    assert(doc_.graph().data().exits().alive(id));
    auto nodeId = doc_.graph().data().exits().get(id)->at();
    removeEntryLayoutsTx(tx, nodeId);
    graphEditor_.setExitLanesTx(tx, id, std::move(specs));
}

void EditorContext::removeNodeTx(Transaction& tx, NodeId id) {
    assert(doc_.graph().data().nodes().alive(id));
    signalEditor_.removeNodeSignalsTx(tx, id);
    removeEntryLayoutsTx(tx, id);
    graphEditor_.removeNodeTx(tx, id);
}

void EditorContext::removeEntryLayoutTx(Transaction& tx, EntryId id) {
    auto* entry = doc_.graph().data().entries().get(id);
    NodeId at = entry->at();
    auto clustersCopy = entry->clusters();
    auto movementsCopy = entry->movements();

    for (auto m : movementsCopy) demandEditor_.onMovementRemovedTx(tx, m);

    signalEditor_.onEntryLayoutClearedTx(tx, at, clustersCopy, movementsCopy);
    graphEditor_.clearEntryLayoutTx(tx, id);
}

void EditorContext::removeEntryLayoutsTx(Transaction& tx, NodeId id) {
    auto node = doc_.graph().data().nodes().get(id);
    for (auto eId : node->entries()) {
        removeEntryLayoutTx(tx, eId);
    }
}
