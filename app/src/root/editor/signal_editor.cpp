#include "signal_editor.hpp"

#include <algorithm>
#include <road_network/graph.hpp>
#include <road_network/signal/data/signal_group.hpp>
#include <unordered_set>
#include <vector>

#include "bus/network_event_bus.hpp"
#include "bus/network_events.hpp"
#include "road_network/util/ids.hpp"
#include "root/command/solve_priority.hpp"
#include "root/command/transaction.hpp"

SignalEditor::SignalEditor(Signal& signal, const Graph& graph,
                           NetworkEventBus& bus)
    : signal_(signal), graph_(graph), bus_(bus) {}

//
//  INTERSECTION TX
//

IntersectionId SignalEditor::createIntersectionTx(
    Transaction& tx, const std::vector<NodeId>& nodes, std::string name) {
    auto mut = signal_.createIntersection(std::move(name));
    IntersectionId id = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.intersectionRemoved,
                  IntersectionRemoved{id});
    tx.recordEvent(bus_.intersectionCreated, IntersectionCreated{id});

    for (NodeId node : nodes) addNodeToIntersectionTx(tx, id, node);
    return id;
}

void SignalEditor::addNodeToIntersectionTx(Transaction& tx, IntersectionId id,
                                           NodeId node) {
    auto mut = signal_.addNode(id, node);
    tx.recordUndo(std::move(mut.undo), bus_.intersectionNodesChanged,
                  IntersectionNodesChanged{id});
    tx.recordEvent(bus_.intersectionNodesChanged,
                   IntersectionNodesChanged{id});
}

void SignalEditor::removeNodeFromIntersectionTx(Transaction& tx,
                                                IntersectionId id,
                                                NodeId node) {
    auto mut = signal_.removeNode(id, node);
    tx.recordUndo(std::move(mut.undo), bus_.intersectionNodesChanged,
                  IntersectionNodesChanged{id});
    tx.recordEvent(bus_.intersectionNodesChanged,
                   IntersectionNodesChanged{id});
}

void SignalEditor::removeIntersectionTx(Transaction& tx, IntersectionId id) {
    auto nodesCopy = signal_.data().intersections().get(id)->nodes();
    for (NodeId node : nodesCopy) removeNodeFromIntersectionTx(tx, id, node);

    auto mut = signal_.removeIntersection(id);
    tx.recordUndo(std::move(mut.undo), bus_.intersectionCreated,
                  IntersectionCreated{id});
    tx.recordEvent(bus_.intersectionRemoved, IntersectionRemoved{id});
}

//
//  CONFIG TX
//

SignalConfigId SignalEditor::createConfigTx(Transaction& tx, IntersectionId at,
                                            std::string name) {
    auto mut = signal_.createConfig(at, std::move(name));
    SignalConfigId id = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.signalConfigRemoved,
                  SignalConfigRemoved{id, at});
    tx.recordEvent(bus_.signalConfigCreated, SignalConfigCreated{id, at});
    return id;
}

void SignalEditor::renameConfigTx(Transaction& tx, SignalConfigId id,
                                  std::string name) {
    auto mut = signal_.renameConfig(id, std::move(name));
    tx.recordUndo(std::move(mut.undo), bus_.signalConfigRenamed,
                  SignalConfigRenamed{id, mut.at});
    tx.recordEvent(bus_.signalConfigRenamed, SignalConfigRenamed{id, mut.at});
}

void SignalEditor::removeConfigTx(Transaction& tx, SignalConfigId id) {
    auto sgsCopy = signal_.data().configs().get(id)->signalGroups();
    for (SignalGroupId sgId : sgsCopy) removeSignalGroupTx(tx, sgId);

    auto mut = signal_.removeConfig(id);
    IntersectionId at = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.signalConfigCreated,
                  SignalConfigCreated{id, at});
    tx.recordEvent(bus_.signalConfigRemoved, SignalConfigRemoved{id, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

//
//  SIGNAL GROUP TX
//

std::expected<SignalGroupId, SignalGroupError>
SignalEditor::addMovementGroupTx(Transaction& tx, SignalConfigId config,
                                 std::vector<ClusterId> clusters,
                                 bool isProtected, int intervalMin) {
    const auto* cfg = signal_.data().configs().get(config);
    for (SignalGroupId sgId : cfg->signalGroups()) {
        const auto* sg = signal_.data().signalGroups().get(sgId);
        const auto* mg = std::get_if<MovementSignalGroup>(sg);
        if (!mg) continue;
        for (ClusterId c : clusters)
            if (std::ranges::find(mg->clusters(), c) != mg->clusters().end())
                return std::unexpected(
                    SignalGroupError::DuplicateClusterMembership);
    }

    std::vector<MovementId> movements;
    for (auto c : clusters) {
        const auto& movs = graph_.data().clusters().get(c)->movements();
        movements.insert(movements.end(), movs.begin(), movs.end());
    }
    return addSignalGroupTx(
        tx, signal_.createMovementGroup(config, std::move(clusters),
                                        std::move(movements), isProtected,
                                        intervalMin));
}

std::expected<SignalGroupId, SignalGroupError>
SignalEditor::addCrosswalkGroupTx(Transaction& tx, SignalConfigId config,
                                  std::vector<CrosswalkId> crosswalks,
                                  int intervalMin) {
    const auto* cfg = signal_.data().configs().get(config);
    for (SignalGroupId sgId : cfg->signalGroups()) {
        const auto* sg = signal_.data().signalGroups().get(sgId);
        const auto* cg = std::get_if<CrosswalkSignalGroup>(sg);
        if (!cg) continue;
        for (CrosswalkId c : crosswalks)
            if (std::ranges::find(cg->crosswalks(), c) !=
                cg->crosswalks().end())
                return std::unexpected(
                    SignalGroupError::DuplicateCrosswalkMembership);
    }
    return addSignalGroupTx(tx, signal_.createCrosswalkGroup(
                                    config, std::move(crosswalks),
                                    intervalMin));
}

std::expected<SignalGroupId, SignalGroupError>
SignalEditor::addTurnArrowGroupTx(Transaction& tx, SignalConfigId config,
                                  MovementId movement, int intervalMin) {
    const auto* turn = graph_.derived().turnTypes().get(movement);
    if (!turn || *turn != TurnType::RIGHT)
        return std::unexpected(
            SignalGroupError::MovementNotEligibleForTurnArrow);

    const auto* cfg = signal_.data().configs().get(config);
    for (SignalGroupId sgId : cfg->signalGroups()) {
        const auto* sg = signal_.data().signalGroups().get(sgId);
        const auto* tg = std::get_if<ConditionalTurnArrowSignalGroup>(sg);
        if (tg && tg->movement() == movement)
            return std::unexpected(
                SignalGroupError::DuplicateTurnArrowForMovement);
    }
    return addSignalGroupTx(
        tx, signal_.createTurnArrowGroup(config, movement, intervalMin));
}

void SignalEditor::setIntervalMinTx(Transaction& tx, SignalGroupId id,
                                    int min) {
    auto mut = signal_.setIntervalMin(id, min);
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupChanged,
                  SignalGroupChanged{id, mut.config, mut.at});
    tx.recordEvent(bus_.signalGroupChanged,
                   SignalGroupChanged{id, mut.config, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

void SignalEditor::setProtectedTx(Transaction& tx, SignalGroupId id,
                                  bool isProtected) {
    auto mut = signal_.setProtected(id, isProtected);
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupChanged,
                  SignalGroupChanged{id, mut.config, mut.at});
    tx.recordEvent(bus_.signalGroupChanged,
                   SignalGroupChanged{id, mut.config, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

void SignalEditor::markConfigStaleTx(Transaction& tx, SignalConfigId id) {
    signal_.markStale(id);
    tx.recordUndoSilent([this, id] { signal_.markStale(id); });
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

void SignalEditor::removeSignalGroupTx(Transaction& tx, SignalGroupId id) {
    auto mut = signal_.removeSignalGroup(id);
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupCreated,
                  SignalGroupCreated{id, mut.config, mut.at});
    tx.recordEvent(bus_.signalGroupRemoved,
                   SignalGroupRemoved{id, mut.config, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

//
//  CONST QUERIES FOR EDITOR CONTEXT
//

std::optional<IntersectionId> SignalEditor::intersectionAt(NodeId id) const {
    return signal_.data().intersectionAt(id);
}

std::vector<SignalConfigId> SignalEditor::configsOf(IntersectionId id) const {
    const auto* in = signal_.data().intersections().get(id);
    if (!in) return {};
    return in->configs();
}

namespace {
std::vector<SignalConfigId> configsAtNode(const Signal& signal, NodeId at) {
    auto intersection = signal.data().intersectionAt(at);
    if (!intersection) return {};
    return signal.data().intersections().get(*intersection)->configs();
}
}  // namespace

std::vector<SignalGroupId> SignalEditor::groupsReferencing(
    NodeId at, const std::vector<ClusterId>& clusters,
    const std::vector<MovementId>& movements) const {
    std::vector<SignalGroupId> result;
    for (SignalConfigId cfgId : configsAtNode(signal_, at)) {
        const auto* cfg = signal_.data().configs().get(cfgId);
        for (SignalGroupId sgId : cfg->signalGroups()) {
            const auto* sg = signal_.data().signalGroups().get(sgId);
            bool match = false;

            if (const auto* mov = std::get_if<MovementSignalGroup>(sg)) {
                match =
                    std::ranges::any_of(mov->clusters(), [&](ClusterId cid) {
                        return std::ranges::find(clusters, cid) !=
                               clusters.end();
                    });
            } else if (const auto* turn =
                           std::get_if<ConditionalTurnArrowSignalGroup>(sg)) {
                match = std::ranges::find(movements, turn->movement()) !=
                        movements.end();
            }

            if (match) result.push_back(sgId);
        }
    }
    return result;
}

std::vector<SignalGroupId> SignalEditor::groupsReferencingCrosswalk(
    NodeId at, CrosswalkId id) const {
    std::vector<SignalGroupId> result;
    for (SignalConfigId cfgId : configsAtNode(signal_, at)) {
        const auto* cfg = signal_.data().configs().get(cfgId);
        for (SignalGroupId sgId : cfg->signalGroups()) {
            const auto* sg = signal_.data().signalGroups().get(sgId);
            if (const auto* g = std::get_if<CrosswalkSignalGroup>(sg)) {
                if (std::ranges::find(g->crosswalks(), id) !=
                    g->crosswalks().end())
                    result.push_back(sgId);
            }
        }
    }
    return result;
}

std::vector<MovementId> SignalEditor::eligibleTurnArrowMovements(
    SignalConfigId config) const {
    std::vector<MovementId> result;
    const auto* cfg = signal_.data().configs().get(config);
    if (!cfg) return result;
    const auto* intersection = signal_.data().intersections().get(cfg->at());
    if (!intersection) return result;

    std::unordered_set<MovementId> taken;
    for (SignalGroupId sgId : cfg->signalGroups()) {
        const auto* sg = signal_.data().signalGroups().get(sgId);
        if (const auto* tg = std::get_if<ConditionalTurnArrowSignalGroup>(sg))
            taken.insert(tg->movement());
    }

    for (NodeId nId : intersection->nodes()) {
        const auto* node = graph_.data().nodes().get(nId);
        if (!node) continue;
        for (EntryId eId : node->entries()) {
            for (MovementId mId :
                 graph_.data().entries().get(eId)->movements()) {
                const auto* turn = graph_.derived().turnTypes().get(mId);
                if (turn && *turn == TurnType::RIGHT && !taken.contains(mId))
                    result.push_back(mId);
            }
        }
    }
    return result;
}

//
//  PRIVATE
//

SignalGroupId SignalEditor::addSignalGroupTx(Transaction& tx,
                                             Signal::GroupMutation mut) {
    SignalGroupId id = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupRemoved,
                  SignalGroupRemoved{id, mut.config, mut.at});
    tx.recordEvent(bus_.signalGroupCreated,
                   SignalGroupCreated{id, mut.config, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
    return id;
}
