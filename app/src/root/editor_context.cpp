#include "editor_context.hpp"

#include <cassert>
#include <expected>
#include <signal_optimizer/optimizer.hpp>
#include <string>
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
      controlEditor_(doc.control(), doc.signal(), doc.graph(), bus),
      destroyQuery_(doc, signalEditor_),
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
    NodeId at = entry->at();

    for (auto m : entry->movements()) {
        demandEditor_.onMovementRemovedTx(tx, m);
    }
    auto movements = graphEditor_.setEntryLayoutTx(tx, id, std::move(specs),
                                                   std::move(shared));
    markSignalControlStaleAtNodeTx(tx, at);
    return movements;
}

void EditorContext::removeCrosswalkTx(Transaction& tx, CrosswalkId id) {
    assert(doc_.graph().data().crosswalks().alive(id));
    auto* cw = doc_.graph().data().crosswalks().get(id);
    NodeId at = doc_.graph().data().crosswalkSeries().get(cw->parent())->at();
    for (auto sg : signalEditor_.groupsReferencingCrosswalk(at, id))
        removeSignalGroupTx(tx, sg);
    graphEditor_.removeCrosswalkTx(tx, id);
    markSignalControlStaleAtNodeTx(tx, at);
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
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::removeEntryTx(Transaction& tx, EntryId id) {
    assert(doc_.graph().data().entries().alive(id));
    auto& data = doc_.graph().data();
    auto nodeId = data.entries().get(id)->at();
    removeEntryLayoutsTx(tx, nodeId);
    graphEditor_.removeEntryTx(tx, id);
    markSignalControlStaleAtNodeTx(tx, nodeId);
}

void EditorContext::removeExitTx(Transaction& tx, ExitId id) {
    assert(doc_.graph().data().exits().alive(id));
    auto& data = doc_.graph().data();
    auto nodeId = data.exits().get(id)->at();
    removeEntryLayoutsTx(tx, nodeId);
    graphEditor_.removeExitTx(tx, id);
    markSignalControlStaleAtNodeTx(tx, nodeId);
}

void EditorContext::setEntryLanesTx(Transaction& tx, EntryId id,
                                    std::vector<EntryLaneSpec> specs) {
    assert(doc_.graph().data().entries().alive(id));
    NodeId at = doc_.graph().data().entries().get(id)->at();
    removeEntryLayoutTx(tx, id);
    graphEditor_.setEntryLanesTx(tx, id, std::move(specs));
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::setExitLanesTx(Transaction& tx, ExitId id,
                                   std::vector<ExitLaneSpec> specs) {
    assert(doc_.graph().data().exits().alive(id));
    auto nodeId = doc_.graph().data().exits().get(id)->at();
    removeEntryLayoutsTx(tx, nodeId);
    graphEditor_.setExitLanesTx(tx, id, std::move(specs));
    markSignalControlStaleAtNodeTx(tx, nodeId);
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
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::removeEntryLayoutsTx(Transaction& tx, NodeId id) {
    auto node = doc_.graph().data().nodes().get(id);
    for (auto eId : node->entries()) {
        removeEntryLayoutTx(tx, eId);
    }
}

//
//  GRAPH PARAMETER EDITS — preserve identity, revalidate downstream
//

void EditorContext::moveEntryTx(Transaction& tx, EntryId id, Position position,
                                Vector2 heading) {
    assert(doc_.graph().data().entries().alive(id));
    NodeId at = doc_.graph().data().entries().get(id)->at();
    graphEditor_.moveEntryTx(tx, id, position, heading);
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::moveExitTx(Transaction& tx, ExitId id, Position position,
                               Vector2 heading) {
    assert(doc_.graph().data().exits().alive(id));
    NodeId at = doc_.graph().data().exits().get(id)->at();
    graphEditor_.moveExitTx(tx, id, position, heading);
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::setEntrySpeedLimitTx(Transaction& tx, EntryId id,
                                         float speedLimit) {
    assert(doc_.graph().data().entries().alive(id));
    NodeId at = doc_.graph().data().entries().get(id)->at();
    graphEditor_.setEntrySpeedLimitTx(tx, id, speedLimit);
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::setExitSpeedLimitTx(Transaction& tx, ExitId id,
                                        float speedLimit) {
    assert(doc_.graph().data().exits().alive(id));
    NodeId at = doc_.graph().data().exits().get(id)->at();
    graphEditor_.setExitSpeedLimitTx(tx, id, speedLimit);
    markSignalControlStaleAtNodeTx(tx, at);
}

void EditorContext::setCrosswalkSeriesWalkSpeedTx(Transaction& tx,
                                                  CrosswalkSeriesId id,
                                                  float speed) {
    assert(doc_.graph().data().crosswalkSeries().alive(id));
    NodeId at = doc_.graph().data().crosswalkSeries().get(id)->at();
    graphEditor_.setCrosswalkSeriesWalkSpeedTx(tx, id, speed);
    markSignalControlStaleAtNodeTx(tx, at);
}

CrosswalkId EditorContext::createCrosswalkTx(Transaction& tx,
                                             CrosswalkSeriesId parent,
                                             Line line) {
    assert(doc_.graph().data().crosswalkSeries().alive(parent));
    NodeId at = doc_.graph().data().crosswalkSeries().get(parent)->at();
    CrosswalkId id = graphEditor_.createCrosswalkTx(tx, parent, line);
    markSignalControlStaleAtNodeTx(tx, at);
    return id;
}

namespace {
NodeId crosswalkNode(const GraphData& data, CrosswalkId id) {
    return data.crosswalkSeries().get(data.crosswalks().get(id)->parent())->at();
}
}  // namespace

void EditorContext::setCrosswalkLineTx(Transaction& tx, CrosswalkId id,
                                       Line line) {
    assert(doc_.graph().data().crosswalks().alive(id));
    graphEditor_.setCrosswalkLineTx(tx, id, line);
    markSignalControlStaleAtNodeTx(tx, crosswalkNode(doc_.graph().data(), id));
}

void EditorContext::setCrosswalkEdgeTx(Transaction& tx, CrosswalkId id,
                                       Crosswalk::SegmentParams edge) {
    assert(doc_.graph().data().crosswalks().alive(id));
    graphEditor_.setCrosswalkEdgeTx(tx, id, edge);
    markSignalControlStaleAtNodeTx(tx, crosswalkNode(doc_.graph().data(), id));
}

void EditorContext::setCrosswalkSplitTx(Transaction& tx, CrosswalkId id,
                                        Crosswalk::SegmentParams split) {
    assert(doc_.graph().data().crosswalks().alive(id));
    graphEditor_.setCrosswalkSplitTx(tx, id, split);
    markSignalControlStaleAtNodeTx(tx, crosswalkNode(doc_.graph().data(), id));
}

void EditorContext::setCrosswalkTypeTx(Transaction& tx, CrosswalkId id,
                                       CrosswalkType type) {
    assert(doc_.graph().data().crosswalks().alive(id));
    graphEditor_.setCrosswalkTypeTx(tx, id, type);
    markSignalControlStaleAtNodeTx(tx, crosswalkNode(doc_.graph().data(), id));
}

void EditorContext::setStreamPermittedOverrideTx(Transaction& tx, NodeId node,
                                                 StreamId evac, StreamId appr,
                                                 bool permitted) {
    graphEditor_.setStreamPermittedOverrideTx(tx, node, evac, appr, permitted);
    markSignalControlStaleAtNodeTx(tx, node);
}

void EditorContext::eraseStreamPermittedOverrideTx(Transaction& tx, NodeId node,
                                                   StreamId evac,
                                                   StreamId appr) {
    graphEditor_.eraseStreamPermittedOverrideTx(tx, node, evac, appr);
    markSignalControlStaleAtNodeTx(tx, node);
}

void EditorContext::setStreamIntergreenOverrideTx(Transaction& tx, NodeId node,
                                                  StreamId evac, StreamId appr,
                                                  int t_maxEvAp,
                                                  int t_maxApEv) {
    graphEditor_.setStreamIntergreenOverrideTx(tx, node, evac, appr, t_maxEvAp,
                                               t_maxApEv);
    markSignalControlStaleAtNodeTx(tx, node);
}

void EditorContext::eraseStreamIntergreenOverrideTx(Transaction& tx,
                                                    NodeId node, StreamId evac,
                                                    StreamId appr) {
    graphEditor_.eraseStreamIntergreenOverrideTx(tx, node, evac, appr);
    markSignalControlStaleAtNodeTx(tx, node);
}

void EditorContext::markSignalControlStaleAtNodeTx(Transaction& tx,
                                                   NodeId id) {
    auto intersection = signalEditor_.intersectionAt(id);
    if (!intersection) return;
    for (SignalConfigId cfg : signalEditor_.configsOf(*intersection)) {
        signalEditor_.markConfigStaleTx(tx, cfg);
        controlEditor_.onConfigParamsChangedTx(tx, cfg);
    }
}

//
//  SIGNAL / CONTROL CASCADES
//

std::expected<SignalGroupId, SignalGroupError>
EditorContext::addMovementGroupTx(Transaction& tx, SignalConfigId config,
                                  std::vector<ClusterId> clusters,
                                  bool isProtected, int intervalMin) {
    auto id = signalEditor_.addMovementGroupTx(tx, config, std::move(clusters),
                                               isProtected, intervalMin);
    if (id) controlEditor_.onSignalGroupCreatedTx(tx, config, *id);
    return id;
}

std::expected<SignalGroupId, SignalGroupError>
EditorContext::addCrosswalkGroupTx(Transaction& tx, SignalConfigId config,
                                   std::vector<CrosswalkId> crosswalks,
                                   int intervalMin) {
    auto id = signalEditor_.addCrosswalkGroupTx(tx, config,
                                                std::move(crosswalks),
                                                intervalMin);
    if (id) controlEditor_.onSignalGroupCreatedTx(tx, config, *id);
    return id;
}

std::expected<SignalGroupId, SignalGroupError>
EditorContext::addTurnArrowGroupTx(Transaction& tx, SignalConfigId config,
                                   MovementId movement, int intervalMin) {
    auto id =
        signalEditor_.addTurnArrowGroupTx(tx, config, movement, intervalMin);
    if (id) controlEditor_.onSignalGroupCreatedTx(tx, config, *id);
    return id;
}

void EditorContext::setSignalGroupIntervalMinTx(Transaction& tx,
                                                SignalGroupId id, int min) {
    SignalConfigId config =
        std::visit([](const auto& g) { return g.config(); },
                   *doc_.signal().data().signalGroups().get(id));
    signalEditor_.setIntervalMinTx(tx, id, min);
    controlEditor_.onConfigParamsChangedTx(tx, config);
}

void EditorContext::setSignalGroupProtectedTx(Transaction& tx,
                                              SignalGroupId id,
                                              bool isProtected) {
    SignalConfigId config =
        std::visit([](const auto& g) { return g.config(); },
                   *doc_.signal().data().signalGroups().get(id));
    signalEditor_.setProtectedTx(tx, id, isProtected);
    controlEditor_.onConfigParamsChangedTx(tx, config);
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

    // dedupe the config name against the intersection's existing configs
    auto nameTaken = [&](const std::string& name) {
        for (SignalConfigId cfgId : intersection->configs())
            if (doc_.signal().data().configs().get(cfgId)->name() == name)
                return true;
        return false;
    };
    std::string name = "optimized";
    for (int n = 2; nameTaken(name); ++n)
        name = "optimized " + std::to_string(n);

    SignalConfigId configId =
        signalEditor_.createConfigTx(tx, at, std::move(name));

    std::vector<SignalGroupId> sgIds;
    for (const auto& mg : result->mgResults) {
        auto id = addMovementGroupTx(tx, configId, mg.clusters, mg.isProtected,
                                     mg.minGreen);
        assert(id && "optimizer produced overlapping movement groups");
        if (!id) return std::nullopt;
        sgIds.push_back(*id);
    }

    for (const auto& cg : result->cgResults) {
        auto id = addCrosswalkGroupTx(tx, configId, cg.crosswalks, cg.minGreen);
        assert(id && "optimizer produced overlapping crosswalk groups");
        if (!id) return std::nullopt;
        sgIds.push_back(*id);
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
