#include "signal_editor.hpp"

#include <algorithm>
#include <road_network/graph.hpp>
#include <road_network/signal/data/signal_group.hpp>
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
//  CONFIG TX
//

SignalConfigId SignalEditor::createConfigTx(Transaction& tx, NodeId at,
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
    NodeId at = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.signalConfigCreated,
                  SignalConfigCreated{id, at});
    tx.recordEvent(bus_.signalConfigRemoved, SignalConfigRemoved{id, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

//
//  SIGNAL GROUP TX
//

SignalGroupId SignalEditor::addMovementGroupTx(Transaction& tx,
                                               SignalConfigId config,
                                               std::vector<ClusterId> clusters,
                                               bool isProtected,
                                               int intervalMin) {
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

SignalGroupId SignalEditor::addCrosswalkGroupTx(
    Transaction& tx, SignalConfigId config,
    std::vector<CrosswalkId> crosswalks, int intervalMin) {
    return addSignalGroupTx(tx, signal_.createCrosswalkGroup(
                                    config, std::move(crosswalks),
                                    intervalMin));
}

SignalGroupId SignalEditor::addTurnArrowGroupTx(Transaction& tx,
                                                SignalConfigId config,
                                                MovementId movement,
                                                int intervalMin) {
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

std::vector<SignalConfigId> SignalEditor::configsAt(NodeId id) const {
    const auto* ns = signal_.data().nodeSignals().get(id);
    if (!ns) return {};
    return ns->configs;
}

std::vector<SignalGroupId> SignalEditor::groupsReferencing(
    NodeId at, const std::vector<ClusterId>& clusters,
    const std::vector<MovementId>& movements) const {
    std::vector<SignalGroupId> result;
    for (SignalConfigId cfgId : configsAt(at)) {
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
    for (SignalConfigId cfgId : configsAt(at)) {
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
