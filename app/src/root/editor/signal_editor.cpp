#include "signal_editor.hpp"

#include <algorithm>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>
#include <road_network/signal/data/signal_group.hpp>
#include <signal_optimizer/optimizer.hpp>
#include <vector>

#include "bus/network_event_bus.hpp"
#include "bus/network_events.hpp"
#include "road_network/util/ids.hpp"
#include "root/command/solve_priority.hpp"
#include "root/command/transaction.hpp"

SignalEditor::SignalEditor(Signal& signal, const Graph& graph,
                           const Demand& demand, NetworkEventBus& bus)
    : signal_(signal), graph_(graph), demand_(demand), bus_(bus) {}

//
//  SIGNAL GROUP TX
//

SignalGroupId SignalEditor::addMovementGroupTx(Transaction& tx, NodeId at,
                                               std::vector<ClusterId> clusters,
                                               bool isProtected,
                                               int intervalMin) {
    std::vector<MovementId> movements;
    for (auto c : clusters) {
        const auto& movs = graph_.data().clusters().get(c)->movements();
        movements.insert(movements.end(), movs.begin(), movs.end());
    }
    return addSignalGroupTx(
        tx, signal_.createMovementGroup(at, std::move(clusters),
                                        std::move(movements), isProtected,
                                        intervalMin));
}

SignalGroupId SignalEditor::addCrosswalkGroupTx(
    Transaction& tx, NodeId at, std::vector<CrosswalkId> crosswalks,
    int intervalMin) {
    return addSignalGroupTx(tx, signal_.createCrosswalkGroup(
                                    at, std::move(crosswalks), intervalMin));
}

SignalGroupId SignalEditor::addTurnArrowGroupTx(Transaction& tx, NodeId at,
                                                MovementId movement,
                                                int intervalMin) {
    return addSignalGroupTx(
        tx, signal_.createTurnArrowGroup(at, movement, intervalMin));
}

void SignalEditor::setIntervalMinTx(Transaction& tx, SignalGroupId id,
                                    int min) {
    auto mut = signal_.setIntervalMin(id, min);
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupChanged,
                  SignalGroupChanged{id, mut.at});
    tx.recordEvent(bus_.signalGroupChanged, SignalGroupChanged{id, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

void SignalEditor::setProtectedTx(Transaction& tx, SignalGroupId id,
                                  bool isProtected) {
    auto mut = signal_.setProtected(id, isProtected);
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupChanged,
                  SignalGroupChanged{id, mut.at});
    tx.recordEvent(bus_.signalGroupChanged, SignalGroupChanged{id, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

//
//  TIMING LIFECYCLE TX
//

TimingId SignalEditor::createTimingTx(Transaction& tx, NodeId at,
                                      int cycleLength) {
    auto mut = signal_.createTiming(at, cycleLength);
    TimingId id = mut.id;
    tx.recordUndo(std::move(mut.undo), bus_.timingRemoved,
                  TimingRemoved{id, at});
    tx.recordEvent(bus_.timingCreated, TimingCreated{id, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });

    if (const auto* ns = signal_.data().nodeSignals().get(at)) {
        for (SignalGroupId sgId : ns->signalGroups) {
            int minIntervalLength =
                std::visit([](const auto& x) { return x.intervalMin(); },
                           *signal_.data().signalGroups().get(sgId));
            createSequenceTx(tx, id, sgId, minIntervalLength, at);
        }
    }

    return id;
}

void SignalEditor::removeTimingTx(Transaction& tx, TimingId id) {
    NodeId at = signal_.data().timings().get(id)->at();
    auto mut = signal_.removeTiming(id);
    tx.recordUndo(std::move(mut.undo), bus_.timingCreated,
                  TimingCreated{id, at});
    tx.recordEvent(bus_.timingRemoved, TimingRemoved{id, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

//
//  SEQUENCE TX
//

void SignalEditor::setAlwaysActiveTx(Transaction& tx, TimingId timingId,
                                     SignalGroupId sgId) {
    auto mut = signal_.setAlwaysActive(timingId, sgId);
    tx.recordUndo(std::move(mut.undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, mut.at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, mut.at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

std::expected<void, TimingError> SignalEditor::setIntervalsTx(
    Transaction& tx, TimingId timingId, SignalGroupId sgId,
    std::vector<Interval> intervals) {
    auto res = signal_.setIntervals(timingId, sgId, std::move(intervals));
    if (!res) return std::unexpected(res.error());

    tx.recordUndo(std::move(res->undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, res->at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, res->at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
    return {};
}

std::expected<void, TimingWarning> SignalEditor::setMinIntervalLengthTx(
    Transaction& tx, TimingId timingId, SignalGroupId sgId, int min) {
    auto res = signal_.setMinIntervalLength(timingId, sgId, min);
    if (!res) return std::unexpected(res.error());

    tx.recordUndo(std::move(res->undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, res->at});
    tx.recordEvent(bus_.sequenceChanged,
                   SequenceChanged{timingId, sgId, res->at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
    return {};
}

//
//  OPTIMIZER TX
//

std::optional<TimingId> SignalEditor::runOptimizerTx(Transaction& tx, NodeId at,
                                                     int cycleLength) {
    auto result = signal_optimizer::optimize(at, graph_, demand_, cycleLength);
    if (!result) return std::nullopt;

    removeNodeSignalsTx(tx, at);

    std::vector<SignalGroupId> sgIds;
    for (const auto& mg : result->mgResults) {
        auto id = addMovementGroupTx(tx, at, mg.clusters, mg.isProtected, 8);
        sgIds.push_back(id);
    }

    for (const auto& cg : result->cgResults) {
        int intervalMin = 0;
        if (const auto* s = std::get_if<SignalSequence>(&cg.sequence))
            intervalMin = s->minIntervalLength();
        auto id = addCrosswalkGroupTx(tx, at, cg.crosswalks, intervalMin);
        sgIds.push_back(id);
    }

    TimingId timingId = createTimingTx(tx, at, cycleLength);

    auto applySequence = [&](SignalGroupId sgId, const Sequence& seq) {
        const auto* s = std::get_if<SignalSequence>(&seq);
        if (!s || s->intervals().empty()) return;
        auto r = signal_.setIntervals(timingId, sgId, s->intervals());
        if (r) {
            tx.recordUndo(std::move(r->undo), bus_.sequenceChanged,
                          SequenceChanged{timingId, sgId, at});
            tx.recordEvent(bus_.sequenceChanged,
                           SequenceChanged{timingId, sgId, at});
            tx.addSolve(SolvePriority::SIGNAL,
                        [this] { signal_.solve(graph_.derived()); });
        }
    };

    size_t idx = 0;
    for (const auto& mg : result->mgResults)
        applySequence(sgIds[idx++], mg.sequence);
    for (const auto& cg : result->cgResults)
        applySequence(sgIds[idx++], cg.sequence);
    return timingId;
}

//
//  GRAPH CHANGE HOOKS
//

void SignalEditor::removeNodeSignalsTx(Transaction& tx, NodeId id) {
    const auto* ns = signal_.data().nodeSignals().get(id);
    if (!ns) return;

    auto timingsCopy = ns->timings;
    auto sgsCopy = ns->signalGroups;

    for (TimingId timingId : timingsCopy) removeTimingTx(tx, timingId);
    for (SignalGroupId sgId : sgsCopy) removeSignalGroupTx(tx, sgId);
}

void SignalEditor::onEntryLayoutClearedTx(
    Transaction& tx, NodeId at, const std::vector<ClusterId>& clusters,
    const std::vector<MovementId>& movements) {
    const auto* ns = signal_.data().nodeSignals().get(at);
    if (!ns) return;

    auto sgsCopy = ns->signalGroups;
    for (SignalGroupId sgId : sgsCopy) {
        const auto* sg = signal_.data().signalGroups().get(sgId);
        bool remove = false;

        if (const auto* mov = std::get_if<MovementSignalGroup>(sg)) {
            remove = std::ranges::any_of(mov->clusters(), [&](ClusterId cid) {
                return std::ranges::find(clusters, cid) != clusters.end();
            });
        } else if (const auto* turn =
                       std::get_if<ConditionalTurnArrowSignalGroup>(sg)) {
            remove = std::ranges::find(movements, turn->movement()) !=
                     movements.end();
        }

        if (remove) removeSignalGroupTx(tx, sgId);
    }
}

void SignalEditor::onCrosswalkRemovedTx(Transaction& tx, CrosswalkId id,
                                        NodeId at) {
    const auto* ns = signal_.data().nodeSignals().get(at);
    if (!ns) return;

    auto sgsCopy = ns->signalGroups;
    for (SignalGroupId sgId : sgsCopy) {
        const auto* sg = signal_.data().signalGroups().get(sgId);
        if (const auto* g = std::get_if<CrosswalkSignalGroup>(sg)) {
            if (std::ranges::find(g->crosswalks(), id) != g->crosswalks().end())
                removeSignalGroupTx(tx, sgId);
        }
    }
}

//
//  PRIVATE
//

SignalGroupId SignalEditor::addSignalGroupTx(
    Transaction& tx, Signal::EntityMutation<SignalGroupId> mut) {
    SignalGroupId id = mut.id;
    NodeId at = mut.at;
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupRemoved,
                  SignalGroupRemoved{id, at});
    tx.recordEvent(bus_.signalGroupCreated, SignalGroupCreated{id, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });

    if (const auto* ns = signal_.data().nodeSignals().get(at)) {
        int minIntervalLength =
            std::visit([](const auto& x) { return x.intervalMin(); },
                       *signal_.data().signalGroups().get(id));
        for (TimingId timingId : ns->timings)
            createSequenceTx(tx, timingId, id, minIntervalLength, at);
    }

    return id;
}

void SignalEditor::removeSignalGroupTx(Transaction& tx, SignalGroupId id) {
    NodeId at = std::visit([](const auto& x) { return x.at(); },
                           *signal_.data().signalGroups().get(id));

    if (const auto* ns = signal_.data().nodeSignals().get(at)) {
        for (TimingId timingId : ns->timings) {
            if (signal_.data().timings().get(timingId)->hasSequence(id))
                removeSequenceTx(tx, timingId, id, at);
        }
    }

    auto mut = signal_.removeSignalGroup(id);
    tx.recordUndo(std::move(mut.undo), bus_.signalGroupCreated,
                  SignalGroupCreated{id, at});
    tx.recordEvent(bus_.signalGroupRemoved, SignalGroupRemoved{id, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

void SignalEditor::createSequenceTx(Transaction& tx, TimingId timingId,
                                    SignalGroupId sgId, int minIntervalLength,
                                    NodeId at) {
    auto mut = signal_.createSequence(timingId, sgId, minIntervalLength);
    tx.recordUndo(std::move(mut.undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, at});
    tx.recordEvent(bus_.sequenceChanged, SequenceChanged{timingId, sgId, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}

void SignalEditor::removeSequenceTx(Transaction& tx, TimingId timingId,
                                    SignalGroupId sgId, NodeId at) {
    auto mut = signal_.removeSequence(timingId, sgId);
    tx.recordUndo(std::move(mut.undo), bus_.sequenceChanged,
                  SequenceChanged{timingId, sgId, at});
    tx.recordEvent(bus_.sequenceChanged, SequenceChanged{timingId, sgId, at});
    tx.addSolve(SolvePriority::SIGNAL,
                [this] { signal_.solve(graph_.derived()); });
}
