#include "signal_editor.hpp"

#include <cmath>
#include <common/overloaded.hpp>
#include <expected>
#include <regulations/regulations.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>

#include "bus/impl/dirty_events.hpp"
#include "bus/network_event_bus.hpp"
#include "regulations/types/signal_kind.hpp"
#include "regulations/types/stream_kind.hpp"
#include "road_network/signal/data/crosswalk_signal_group.hpp"
#include "road_network/signal/data/movement_signal_group.hpp"
#include "road_network/util/ids.hpp"
#include "root/editor/util/editor_errors.hpp"
#include "util/apply_tx.hpp"

SignalEditor::SignalEditor(Signal& signal, const Graph& graph,
                           NetworkEventBus& bus, DirtyEventBus& dirty,
                           const Regulations& regulations)
    : signal_(signal),
      graph_(graph),
      bus_(bus),
      dirty_(dirty),
      regulations_(regulations) {
    subs_.add(dirty_.nodeUpdated,
              [this](UpdatedTx<NodeId> e) { onNodeUpdated(e); });
    subs_.add(dirty_.nodeRemoved,
              [this](RemovedTx<NodeId> e) { onNodeRemoved(e); });
    subs_.add(dirty_.entryRemoved,
              [this](RemovedTx<EntryId, NodeId> e) { onEntryRemoved(e); });
    subs_.add(dirty_.movementRemoved, [this](RemovedTx<MovementId, NodeId> e) {
        onMovementRemoved(e);
    });
    subs_.add(
        dirty_.crosswalkUpdated,
        [this](UpdatedTx<CrosswalkId, NodeId> e) { onCrosswalkUpdated(e); });
    subs_.add(
        dirty_.crosswalkRemoved,
        [this](RemovedTx<CrosswalkId, NodeId> e) { onCrosswalkRemoved(e); });
}

std::expected<SignalGroupId, SignalGroupErrors>
SignalEditor::createMovementGroup(Transaction& tx, NodeId node,
                                  std::vector<ClusterId> clusters) {
    assert(!clusters.empty());

    const auto* clFront = graph_.data().clusters().get(clusters.front());
    const auto* mFront =
        graph_.data().movements().get(clFront->movements().front());
    auto type = mFront->params().type;

    std::vector<MovementId> movements;
    for (auto cid : clusters) {
        const auto* cl = graph_.data().clusters().get(cid);
        const auto& clmv = cl->movements();

        for (auto mid : clmv) {
            if (type != graph_.data().movements().get(mid)->params().type)
                return std::unexpected(
                    SignalGroupErrors::CandidateMovementsOfMultipleTypes);
        }

        movements.insert(movements.end(), clmv.begin(), clmv.end());
    }
    auto entry = graph_.data().clusters().get(clusters.front())->at();
    auto kind = toSignalKind(type);
    const auto& min = regulations_.aspects(kind).minGreen;

    auto mut = signal_.createMovementGroup(node, entry, std::move(clusters),
                                           std::move(movements), min.fixed);
    const auto id = mut.id;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    return applyTx(tx, std::move(mut), bus_.signalGroupCreated,
                   Created<SignalGroupId, NodeId>{.id = id, .at = node},
                   bus_.signalGroupRemoved,
                   Removed<SignalGroupId, NodeId>{.id = id, .at = node});
}

std::expected<SignalGroupId, SignalGroupErrors>
SignalEditor::createCrosswalkGroup(Transaction& tx, NodeId node,
                                   std::vector<CrosswalkId> crosswalks) {
    const auto& cwMap = graph_.data().crosswalks();
    auto csid = cwMap.get(crosswalks.front())->parent();

    for (auto cid : crosswalks)
        if (cwMap.get(cid)->parent() != csid)
            return std::unexpected(
                SignalGroupErrors::CandidateCrosswalksOfMultipleSeries);

    const auto* cs = graph_.data().crosswalkSeries().get(csid);
    const SignalKind sigKind = toSignalKind(cs->params().type);
    StreamKind strKind = toStreamKind(cs->params().type);
    const auto& min = regulations_.aspects(sigKind).minGreen;
    double regEvac = regulations_.kinematics(strKind).evacSpeedCeil;
    double evacSpeed =
        cs->params().criticalGreenSpeedOverride.value_or(regEvac);
    const int cross =
        std::ceil(cs->params().criticalGreenPath.length() / evacSpeed);

    auto mut = signal_.createCrosswalkGroup(
        node, std::move(crosswalks), sigKind, std::max(cross, min.fixed));
    const auto id = mut.id;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    return applyTx(tx, std::move(mut), bus_.signalGroupCreated,
                   Created<SignalGroupId, NodeId>{.id = id, .at = node},
                   bus_.signalGroupRemoved,
                   Removed<SignalGroupId, NodeId>{.id = id, .at = node});
}

std::expected<void, SignalGroupErrors> SignalEditor::setSgProtection(
    Transaction& tx, SignalGroupId id, bool request) {
    auto* sg = signal_.data().signalGroups().get(id);
    auto msg = std::get_if<MovementSignalGroup>(sg);
    if (!msg)
        return std::unexpected(
            SignalGroupErrors::ProtectionEditOnNonMovementGroup);

    auto params = msg->params();
    params.requestProtected = request;

    auto mut = signal_.setParams(id, params);
    const auto node = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.signalGroupUpdated,
                  Updated<SignalGroupId, NodeId>{.id = id, .at = node});
    return {};
}

void SignalEditor::setSgIntervalMinFixed(Transaction& tx, SignalGroupId id,
                                         int min) {
    auto* sg = signal_.data().signalGroups().get(id);

    auto mut = std::visit(
        [&](auto& sgv) {
            auto params = sgv.params();
            params.intervalMinFixed = min;
            return signal_.setParams(id, params);
        },
        *sg);

    const auto node = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.signalGroupUpdated,
                  Updated<SignalGroupId, NodeId>{.id = id, .at = node});
}

void SignalEditor::setSgKind(Transaction& tx, SignalGroupId id,
                             SignalKind kind) {
    auto* sg = signal_.data().signalGroups().get(id);
    auto mut = std::visit(
        overloaded{
            [&](const MovementSignalGroup& msg) {
                if (!(kind == SignalKind::VEHICLE || kind == SignalKind::BUS ||
                      kind == SignalKind::TRAM))
                    assert(false);
                auto params = msg.params();
                params.kind = kind;
                return signal_.setParams(id, params);
            },
            [&](const CrosswalkSignalGroup& csg) {
                // A crosswalk group's kind is fixed by its topology; it is
                // never set through the inspector.
                assert(false);
                return signal_.setParams(id, csg.params());
            },
        },
        *sg);
    const auto node = mut.at;
    dirty_.nodeUpdated.publish(UpdatedTx<NodeId>{.id = node, .tx = tx});
    applyTxUpdate(tx, std::move(mut), bus_.signalGroupUpdated,
                  Updated<SignalGroupId, NodeId>{.id = id, .at = node});
}

void SignalEditor::removeSignalGroup(Transaction& tx, SignalGroupId id) {
    const auto node = signal_.parentOf(id);
    auto mut = signal_.removeSignalGroup(id);
    dirty_.signalGroupRemoved.publish(
        RemovedTx<SignalGroupId, NodeId>{.id = id, .at = node, .tx = tx});
    applyTx(tx, std::move(mut), bus_.signalGroupRemoved,
            Removed<SignalGroupId, NodeId>{.id = id, .at = node},
            bus_.signalGroupCreated,
            Created<SignalGroupId, NodeId>{.id = id, .at = node});
}

void SignalEditor::registerSolve(Transaction& tx) {
    reNodes_.clear();
    tx.addSolve(SolvePriority::SIGNAL, {
                                           .rederive = [this] { rederive(); },
                                           .flush = [this] { flush(); },
                                       });
}

void SignalEditor::rederive() {
    auto r = signal_.solve(graph_);
    reNodes_.insert(r.nodes.begin(), r.nodes.end());
}

void SignalEditor::flush() {
    for (auto id : reNodes_) {
        if (graph_.data().nodes().alive(id)) {
            bus_.nodeUpdatedSignal.publish(Updated<NodeId>{.id = id});
        }
    }
    reNodes_.clear();
}

void SignalEditor::onNodeUpdated(UpdatedTx<NodeId> e) {
    e.tx.solveUpto(SolvePriority::SIGNAL);
    signal_.markDirty(e.id);
}

void SignalEditor::onNodeRemoved(RemovedTx<NodeId> e) {
    for (auto id : *signal_.data().nodeGroups().get(e.id)) {
        removeSignalGroup(e.tx, id);
    }
}

void SignalEditor::onEntryRemoved(RemovedTx<EntryId, NodeId> e) {
    std::vector<SignalGroupId> doomed;
    for (auto gid : *signal_.data().nodeGroups().get(e.at)) {
        auto g = signal_.data().signalGroups().get(gid);
        if (const auto* msg = std::get_if<MovementSignalGroup>(g))
            if (msg->entry() == e.id) doomed.push_back(gid);
    }
    for (auto id : doomed) removeSignalGroup(e.tx, id);
}

void SignalEditor::onMovementRemoved(RemovedTx<MovementId, NodeId> e) {
    removeGroupsWithStream(e.tx, StreamId{e.id}, e.at);
}

void SignalEditor::onCrosswalkUpdated(UpdatedTx<CrosswalkId, NodeId> e) {
    removeGroupsWithStream(e.tx, StreamId{e.id}, e.at);
}

void SignalEditor::onCrosswalkRemoved(RemovedTx<CrosswalkId, NodeId> e) {
    removeGroupsWithStream(e.tx, StreamId{e.id}, e.at);
}

void SignalEditor::removeGroupsWithStream(Transaction& tx, StreamId stream,
                                          NodeId at) {
    std::vector<SignalGroupId> doomed;
    for (auto gid : *signal_.data().nodeGroups().get(at)) {
        auto g = signal_.data().signalGroups().get(gid);
        const std::vector<StreamId> streams =
            std::visit([](const auto& sg) { return sg.streams(); }, *g);
        if (std::ranges::find(streams, stream) != streams.end())
            doomed.push_back(gid);
    }
    for (auto id : doomed) removeSignalGroup(tx, id);
}