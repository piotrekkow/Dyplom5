#include "editor_context.hpp"

#include <expected>
#include <signal_optimizer/optimizer.hpp>
#include <tuple>
#include <variant>

#include "bus/network_event_bus.hpp"
#include "command/transaction.hpp"
#include "document.hpp"
#include "road_network/util/ids.hpp"
#include "use_cases/control_use_cases.hpp"
#include "use_cases/demand_use_cases.hpp"
#include "use_cases/graph_use_cases.hpp"
#include "use_cases/signal_use_cases.hpp"

EditorContext::EditorContext(Document& doc, NetworkEventBus& bus)
    : doc_(doc),
      cmdStack_(),
      graphEditor_(doc.graph(), bus),
      demandEditor_(doc.demand(), doc.graph().data(), bus),
      signalEditor_(doc.signal(), doc.graph(), bus),
      controlEditor_(doc.control(), doc.signal(), bus),
      graphUseCases_(
          std::make_unique<GraphUseCases>(graphEditor_, *this, cmdStack_)),
      demandUseCases_(
          std::make_unique<DemandUseCases>(demandEditor_, cmdStack_)),
      signalUseCases_(
          std::make_unique<SignalUseCases>(signalEditor_, *this, cmdStack_)),
      controlUseCases_(
          std::make_unique<ControlUseCases>(controlEditor_, cmdStack_)) {}

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
    for (auto sg : signalEditor_.groupsReferencingCrosswalk(at, id))
        removeSignalGroupTx(tx, sg);
    graphEditor_.removeCrosswalkTx(tx, id);
}

void EditorContext::removeCrosswalkSeriesTx(Transaction& tx,
                                            CrosswalkSeriesId id) {
    assert(doc_.graph().data().crosswalkSeries().alive(id));
    auto* cs = doc_.graph().data().crosswalkSeries().get(id);
    NodeId at = cs->at();
    auto crosswalksCopy = cs->crosswalks();
    for (CrosswalkId cid : crosswalksCopy)
        for (auto sg : signalEditor_.groupsReferencingCrosswalk(at, cid))
            removeSignalGroupTx(tx, sg);
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

    auto intersection = signalEditor_.intersectionAt(id);
    if (intersection &&
        doc_.signal().data().intersections().get(*intersection)->nodes().size() ==
            1) {
        // last node — the intersection dies with it, configs and all
        removeIntersectionTx(tx, *intersection);
        intersection.reset();
    }

    // strip groups referencing this node's crosswalks before the graph
    // removes the crosswalk series (movement groups are handled by
    // removeEntryLayoutsTx)
    if (intersection) {
        const auto* node = doc_.graph().data().nodes().get(id);
        for (CrosswalkSeriesId csId : node->crosswalkSeries()) {
            auto crosswalksCopy =
                doc_.graph().data().crosswalkSeries().get(csId)->crosswalks();
            for (CrosswalkId cwId : crosswalksCopy)
                for (auto sg : signalEditor_.groupsReferencingCrosswalk(id, cwId))
                    removeSignalGroupTx(tx, sg);
        }
    }

    removeEntryLayoutsTx(tx, id);

    if (intersection)
        signalEditor_.removeNodeFromIntersectionTx(tx, *intersection, id);

    graphEditor_.removeNodeTx(tx, id);
}

void EditorContext::removeEntryLayoutTx(Transaction& tx, EntryId id) {
    auto* entry = doc_.graph().data().entries().get(id);
    NodeId at = entry->at();
    auto clustersCopy = entry->clusters();
    auto movementsCopy = entry->movements();

    for (auto m : movementsCopy) demandEditor_.onMovementRemovedTx(tx, m);

    for (auto sg : signalEditor_.groupsReferencing(at, clustersCopy,
                                                   movementsCopy))
        removeSignalGroupTx(tx, sg);
    graphEditor_.clearEntryLayoutTx(tx, id);
}

void EditorContext::removeEntryLayoutsTx(Transaction& tx, NodeId id) {
    auto node = doc_.graph().data().nodes().get(id);
    for (auto eId : node->entries()) {
        removeEntryLayoutTx(tx, eId);
    }
}

//
//  SIGNAL / CONTROL CASCADES
//

SignalGroupId EditorContext::addMovementGroupTx(
    Transaction& tx, SignalConfigId config, std::vector<ClusterId> clusters,
    bool isProtected, int intervalMin) {
    auto id = signalEditor_.addMovementGroupTx(tx, config, std::move(clusters),
                                               isProtected, intervalMin);
    controlEditor_.onSignalGroupCreatedTx(tx, config, id, intervalMin);
    return id;
}

SignalGroupId EditorContext::addCrosswalkGroupTx(
    Transaction& tx, SignalConfigId config, std::vector<CrosswalkId> crosswalks,
    int intervalMin) {
    auto id = signalEditor_.addCrosswalkGroupTx(tx, config,
                                                std::move(crosswalks),
                                                intervalMin);
    controlEditor_.onSignalGroupCreatedTx(tx, config, id, intervalMin);
    return id;
}

SignalGroupId EditorContext::addTurnArrowGroupTx(Transaction& tx,
                                                 SignalConfigId config,
                                                 MovementId movement,
                                                 int intervalMin) {
    auto id =
        signalEditor_.addTurnArrowGroupTx(tx, config, movement, intervalMin);
    controlEditor_.onSignalGroupCreatedTx(tx, config, id, intervalMin);
    return id;
}

void EditorContext::removeSignalGroupTx(Transaction& tx, SignalGroupId id) {
    SignalConfigId config =
        std::visit([](const auto& g) { return g.config(); },
                   *doc_.signal().data().signalGroups().get(id));
    controlEditor_.onSignalGroupRemovedTx(tx, config, id);
    signalEditor_.removeSignalGroupTx(tx, id);
}

void EditorContext::removeSignalConfigTx(Transaction& tx, SignalConfigId id) {
    controlEditor_.onConfigRemovedTx(tx, id);
    signalEditor_.removeConfigTx(tx, id);
}

void EditorContext::removeIntersectionTx(Transaction& tx, IntersectionId id) {
    for (SignalConfigId cfg : signalEditor_.configsOf(id))
        removeSignalConfigTx(tx, cfg);
    signalEditor_.removeIntersectionTx(tx, id);
}

//
//  OPTIMIZER
//

std::optional<EditorContext::OptimizerRunResult> EditorContext::runOptimizerTx(
    Transaction& tx, IntersectionId at, int cycleLength) {
    const auto* intersection = doc_.signal().data().intersections().get(at);
    if (!intersection || intersection->nodes().size() != 1)
        return std::nullopt;  // multi-node optimization not supported yet

    auto result = signal_optimizer::optimize(intersection->nodes().front(),
                                             doc_.graph(), doc_.demand(),
                                             cycleLength);
    if (!result) return std::nullopt;

    SignalConfigId configId = signalEditor_.createConfigTx(tx, at, "optimized");

    std::vector<SignalGroupId> sgIds;
    for (const auto& mg : result->mgResults) {
        auto id =
            addMovementGroupTx(tx, configId, mg.clusters, mg.isProtected, 8);
        sgIds.push_back(id);
    }

    for (const auto& cg : result->cgResults) {
        int intervalMin = 0;
        if (const auto* s = std::get_if<SignalSequence>(&cg.sequence))
            intervalMin = s->minIntervalLength();
        auto id = addCrosswalkGroupTx(tx, configId, cg.crosswalks, intervalMin);
        sgIds.push_back(id);
    }

    TimingId timingId = controlEditor_.createTimingTx(tx, configId, cycleLength);

    auto applySequence = [&](SignalGroupId sgId, const Sequence& seq) {
        const auto* s = std::get_if<SignalSequence>(&seq);
        if (!s || s->intervals().empty()) return;
        std::ignore =
            controlEditor_.setIntervalsTx(tx, timingId, sgId, s->intervals());
    };

    size_t idx = 0;
    for (const auto& mg : result->mgResults)
        applySequence(sgIds[idx++], mg.sequence);
    for (const auto& cg : result->cgResults)
        applySequence(sgIds[idx++], cg.sequence);

    return OptimizerRunResult{configId, timingId};
}
