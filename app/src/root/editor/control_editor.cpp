#include "control_editor.hpp"

#include <control/schema/stage.hpp>
#include <control/transition.hpp>
#include <control/validation.hpp>
#include <expected>
#include <road_network/control.hpp>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>
#include <road_network/util/compute_helpers.hpp>
#include <signal_optimizer/optimizer.hpp>

#include "bus/dirty_event_bus.hpp"
#include "bus/impl/dirty_events.hpp"
#include "bus/network_event_bus.hpp"
#include "road_network/util/ids.hpp"
#include "root/editor/util/editor_errors.hpp"
#include "util/apply_tx.hpp"

ControlEditor::ControlEditor(Control& control, const Signal& signal,
                             const Demand& demand, const Graph& graph,
                             NetworkEventBus& bus, DirtyEventBus& dirty,
                             const Regulations& regulations)
    : control_(control),
      signal_(signal),
      demand_(demand),
      graph_(graph),
      bus_(bus),
      dirty_(dirty),
      regulations_(regulations) {
    subs_.add(dirty_.nodeUpdated,
              [this](UpdatedTx<NodeId> e) { onNodeUpdated(e); });
    subs_.add(dirty_.nodeRemoved,
              [this](RemovedTx<NodeId> e) { onNodeRemoved(e); });
    subs_.add(dirty_.signalGroupCreated,
              [this](CreatedTx<SignalGroupId, NodeId> e) {
                  onSignalGroupCreated(e);
              });
    subs_.add(dirty_.signalGroupRemoved,
              [this](RemovedTx<SignalGroupId, NodeId> e) {
                  onSignalGroupRemoved(e);
              });
}

TimingId ControlEditor::createTiming(Transaction& tx, NodeId at,
                                     std::string name) {
    if (name.empty())
        name = std::to_string(control_.data().timings().end()->id.index());
    auto mut = control_.createTiming(at, std::move(name));
    const auto tid = mut.id;
    return applyTx(tx, std::move(mut), bus_.timingCreated,
                   Created<TimingId, NodeId>{.id = tid, .at = at},
                   bus_.timingRemoved,
                   Removed<TimingId, NodeId>{.id = tid, .at = at});
}
void ControlEditor::setTimingName(Transaction& tx, TimingId id,
                                  std::string name) {
    auto* t = control_.data().timings().get(id);

    auto params = t->params();
    params.name = std::move(name);

    auto mut = control_.setParams(id, params);
    const auto at = mut.at;
    applyTxUpdate(tx, std::move(mut), bus_.timingUpdated,
                  Updated<TimingId, NodeId>{.id = id, .at = at});
}
std::expected<void, TimingErrors> ControlEditor::setTimingSteps(
    Transaction& tx, TimingId id, std::vector<StageStep> steps) {
    {  // validation
        for (int i = 0; i < steps.size(); ++i) {
            const auto* tr =
                control_.data().stageTransitions().get(steps[i].transition());
            if (tr->from() != steps[i].stage())
                return std::unexpected(
                    TimingErrors::StepWithInvalidStageAndTransitionPair);
            size_t next = i == steps.size() - 1 ? 0 : i + 1;
            if (tr->to() != steps[next].stage())
                return std::unexpected(
                    TimingErrors::NextTransitionDoesntMatchStep);
        }
    }  // validation

    auto* t = control_.data().timings().get(id);

    auto params = t->params();
    params.steps = std::move(steps);

    auto mut = control_.setParams(id, params);
    const auto at = mut.at;
    applyTxUpdate(tx, std::move(mut), bus_.timingUpdated,
                  Updated<TimingId, NodeId>{.id = id, .at = at});
    return {};
}

void ControlEditor::removeTiming(Transaction& tx, TimingId id) {
    const auto at = control_.parentOf(id);
    auto mut = control_.removeTiming(id);
    applyTx(tx, std::move(mut), bus_.timingCreated,
            Created<TimingId, NodeId>{.id = id, .at = at}, bus_.timingRemoved,
            Removed<TimingId, NodeId>{.id = id, .at = at});
}

std::expected<StageId, StageErrors> ControlEditor::createStage(
    Transaction& tx, NodeId at, std::unordered_set<SignalGroupId> active) {
    const auto& igMatrix = *signal_.derived().intergreenMatrices().get(at);
    if (!control::stageIntergreenBreaches(igMatrix, active).empty())
        return std::unexpected(
            StageErrors::ContainsPairsDisallowedByIntergreenMatrix);

    auto content = stageBody(std::move(active), at);
    if (!content) return std::unexpected(content.error());

    auto mut = control_.createStage(at, *content);
    const auto sid = mut.id;
    return applyTx(tx, std::move(mut), bus_.stageCreated,
                   Created<StageId, NodeId>{.id = sid, .at = at},
                   bus_.stageRemoved,
                   Removed<StageId, NodeId>{.id = sid, .at = at});
}

std::expected<void, StageErrors> ControlEditor::setStageGroups(
    Transaction& tx, StageId id, std::unordered_set<SignalGroupId> active) {
    const auto at = control_.parentOf(id);
    const auto& igMatrix = *signal_.derived().intergreenMatrices().get(at);
    if (!control::stageIntergreenBreaches(igMatrix, active).empty())
        return std::unexpected(
            StageErrors::ContainsPairsDisallowedByIntergreenMatrix);

    auto content = stageBody(std::move(active), at);
    if (!content) return std::unexpected(content.error());

    auto s = control_.data().stages().get(id);
    auto params = s->params();
    params.core = std::move(*content);

    auto mut = control_.setParams(id, params);
    applyTxUpdate(tx, std::move(mut), bus_.stageUpdated,
                  Updated<StageId, NodeId>{.id = id, .at = at});
    return {};
}

void ControlEditor::removeStage(Transaction& tx, StageId id) {
    const auto at = control_.parentOf(id);
    std::vector<StageTransitionId> doomed;
    for (auto tid : control_.data().nodeControls().get(at)->transitions) {
        const auto* t = control_.data().stageTransitions().get(tid);
        if (t->from() == id || t->to() == id) doomed.push_back(tid);
    }
    for (auto tid : doomed) removeTransition(tx, tid);
    clearTimingsWhereSteps(tx, at,
                           [&](const StageStep& s) { return s.stage() == id; });
    auto mut = control_.removeStage(id);
    applyTx(tx, std::move(mut), bus_.stageCreated,
            Created<StageId, NodeId>{.id = id, .at = at}, bus_.stageRemoved,
            Removed<StageId, NodeId>{.id = id, .at = at});
}

StageTransitionId ControlEditor::createTransition(Transaction& tx, NodeId at,
                                                  StageId from, StageId to) {
    auto mut = control_.createTransition(at, from, to);
    const auto tid = mut.id;
    applyTx(tx, std::move(mut), bus_.stageTransitionCreated,
            Created<StageTransitionId, NodeId>{.id = tid, .at = at},
            bus_.stageTransitionRemoved,
            Removed<StageTransitionId, NodeId>{.id = tid, .at = at});

    const auto* fromStage = control_.data().stages().get(from);
    const auto* toStage = control_.data().stages().get(to);
    const auto* igMatrix = signal_.derived().intergreenMatrices().get(at);
    assert(fromStage && toStage && igMatrix);
    auto weights = compute::groupMaxFlowRatios(signal_, demand_.derived(), at);
    auto gParams = compute::groupParams(signal_, regulations_, at);

    static const ApproachMatrix kNoApproach;
    const auto* apMatrix = signal_.derived().approachMatrices().get(at);

    auto params = control_.data().stageTransitions().get(tid)->params();
    params.core = control::computeTransitionWeighted(
        fromStage->params().core, toStage->params().core, gParams, *igMatrix,
        weights, apMatrix ? *apMatrix : kNoApproach,
        regulations_.enforceApproachRelease());
    applyTxUpdate(tx, control_.setParams(tid, std::move(params)),
                  bus_.stageTransitionUpdated,
                  Updated<StageTransitionId, NodeId>{.id = tid, .at = at});
    return tid;
}

void ControlEditor::removeTransition(Transaction& tx, StageTransitionId id) {
    const auto at = control_.parentOf(id);
    clearTimingsWhereSteps(
        tx, at, [&](const StageStep& s) { return s.transition() == id; });
    auto mut = control_.removeTransition(id);
    applyTx(tx, std::move(mut), bus_.stageTransitionCreated,
            Created<StageTransitionId, NodeId>{.id = id, .at = at},
            bus_.stageTransitionRemoved,
            Removed<StageTransitionId, NodeId>{.id = id, .at = at});
}

void ControlEditor::onNodeUpdated(UpdatedTx<NodeId> e) {
    e.tx.solveUpto(SolvePriority::CONTROL);
    control_.markDirty(e.id);
}

void ControlEditor::onNodeRemoved(RemovedTx<NodeId> e) {
    const auto* nc = control_.data().nodeControls().get(e.id);
    clearNodeControl(e.tx, nc);
}

void ControlEditor::onSignalGroupCreated(CreatedTx<SignalGroupId, NodeId> e) {
    const auto* nc = control_.data().nodeControls().get(e.at);
    clearNodeControl(e.tx, nc);
}

void ControlEditor::onSignalGroupRemoved(RemovedTx<SignalGroupId, NodeId> e) {
    const auto* nc = control_.data().nodeControls().get(e.at);
    clearNodeControl(e.tx, nc);
}

void ControlEditor::clearNodeControl(Transaction& tx, const NodeControl* nc) {
    if (!nc) return;
    auto transitions = nc->transitions;
    auto stages = nc->stages;
    auto timings = nc->timings;
    for (auto id : timings) removeTiming(tx, id);
    for (auto id : transitions) removeTransition(tx, id);
    for (auto id : stages) removeStage(tx, id);
}

void ControlEditor::clearTimingsWhereSteps(Transaction& tx, NodeId at,
                                           auto pred) {
    const auto* nc = control_.data().nodeControls().get(at);
    if (!nc) return;
    std::vector<TimingId> affected;
    for (auto tid : nc->timings)
        if (std::ranges::any_of(
                control_.data().timings().get(tid)->params().steps, pred))
            affected.push_back(tid);
    for (auto tid : affected) {
        auto params = control_.data().timings().get(tid)->params();
        params.steps.clear();
        auto mut = control_.setParams(tid, params);
        applyTxUpdate(tx, std::move(mut), bus_.timingUpdated,
                      Updated<TimingId, NodeId>{.id = tid, .at = at});
    }
}

void ControlEditor::registerSolve(Transaction& tx) {
    reNodes_.clear();
    tx.addSolve(SolvePriority::CONTROL, {.rederive = [this] { rederive(); },
                                         .flush = [this] { flush(); }});
}

void ControlEditor::rederive() {
    auto r = control_.solve(signal_, graph_);
    reNodes_.insert(r.nodes.begin(), r.nodes.end());
}

void ControlEditor::flush() {
    for (auto id : reNodes_) {
        if (graph_.data().nodes().alive(id)) {
            bus_.nodeUpdatedControl.publish(Updated<NodeId>{.id = id});
        }
    }
    reNodes_.clear();
}

std::expected<control::Stage<SignalGroupId>, StageErrors>
ControlEditor::stageBody(std::unordered_set<SignalGroupId> active, NodeId at) {
    control::Stage<SignalGroupId> out;
    const auto* ng = signal_.data().nodeGroups().get(at);
    if (!ng) return out;
    for (auto gid : *ng) {
        const auto* g = signal_.data().signalGroups().get(gid);
        const auto kind =
            std::visit([](const auto& sg) { return sg.params().kind; }, *g);
        const auto aspect = regulations_.aspects(kind);
        out.groups[gid] = {
            .aspect = active.contains(gid) ? aspect.active : aspect.inactive,
            .isActive = active.contains(gid)};
        active.erase(gid);
    }
    if (!active.empty())
        return std::unexpected(StageErrors::GroupsFromOutsideNodeSupplied);
    return out;
}