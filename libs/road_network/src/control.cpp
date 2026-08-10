#include "road_network/control.hpp"

#include "control/timing.hpp"
#include "control/validation.hpp"
#include "derived/control_compute.hpp"
#include "road_network/control/control_derived.hpp"
#include "road_network/graph.hpp"
#include "road_network/signal.hpp"
#include "road_network/util/compute_helpers.hpp"

namespace {
bool emptied(const NodeControl& nc) {
    return nc.stages.empty() && nc.timings.empty() && nc.transitions.empty();
}
}  // namespace

NodeId Control::parentOf(TimingId id) const {
    return data_.timings().get(id)->at();
}
NodeId Control::parentOf(StageId id) const {
    return data_.stages().get(id)->at();
}
NodeId Control::parentOf(StageTransitionId id) const {
    return data_.stageTransitions().get(id)->at();
}
void Control::link(TimingId id) {
    const auto node = parentOf(id);
    data_.nodeControls().ensure(node).timings.push_back(id);
}
void Control::link(StageId id) {
    const auto node = parentOf(id);
    data_.nodeControls().ensure(node).stages.push_back(id);
}
void Control::link(StageTransitionId id) {
    const auto node = parentOf(id);
    data_.nodeControls().ensure(node).transitions.push_back(id);
}
void Control::unlink(TimingId id) {
    const auto node = parentOf(id);
    auto* nc = data_.nodeControls().get(node);
    assert(nc);
    erase(nc->timings, id);
    if (emptied(*nc)) data_.nodeControls().erase(node);
}
void Control::unlink(StageId id) {
    const auto node = parentOf(id);
    auto* nc = data_.nodeControls().get(node);
    assert(nc);
    erase(nc->stages, id);
    if (emptied(*nc)) data_.nodeControls().erase(node);
}
void Control::unlink(StageTransitionId id) {
    const auto node = parentOf(id);
    auto* nc = data_.nodeControls().get(node);
    assert(nc);
    erase(nc->transitions, id);
    if (emptied(*nc)) data_.nodeControls().erase(node);
}

EMut<TimingId, NodeId> Control::createTiming(NodeId at, std::string name) {
    auto id = createChild(data_.timings(), at, std::move(name));
    return {.id = id, .at = at, .undo = [this, id] {
                data_.timings().release(detach(data_.timings(), id));
            }};
}
EMut<TimingId, NodeId> Control::setParams(TimingId id, Timing::Params p) {
    return Base::setParams(data_.timings(), id, std::move(p));
}
EMut<TimingId, NodeId> Control::removeTiming(TimingId id) {
    derived_.plans().erase(id);
    auto at = parentOf(id);
    auto vs = detach(data_.timings(), id);
    return {.id = id, .at = at, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.timings(), std::move(vs));
            }};
}

EMut<StageId, NodeId> Control::createStage(
    NodeId at, control::Stage<SignalGroupId> stage) {
    auto id = createChild(data_.stages(), at, std::move(stage));
    return {.id = id, .at = at, .undo = [this, id] {
                data_.stages().release(detach(data_.stages(), id));
            }};
}
EMut<StageId, NodeId> Control::setParams(StageId id, Stage::Params p) {
    return Base::setParams(data_.stages(), id, std::move(p));
}
EMut<StageId, NodeId> Control::removeStage(StageId id) {
    auto at = parentOf(id);
    auto vs = detach(data_.stages(), id);
    return {.id = id, .at = at, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.stages(), std::move(vs));
            }};
}

EMut<StageTransitionId, NodeId> Control::createTransition(NodeId at,
                                                          StageId from,
                                                          StageId to) {
    auto id = createChild(data_.stageTransitions(), at, from, to);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.stageTransitions().release(
                    detach(data_.stageTransitions(), id));
            }};
}
EMut<StageTransitionId, NodeId> Control::setParams(StageTransitionId id,
                                                   StageTransition::Params p) {
    return Base::setParams(data_.stageTransitions(), id, std::move(p));
}
EMut<StageTransitionId, NodeId> Control::removeTransition(
    StageTransitionId id) {
    auto at = parentOf(id);
    auto vs = detach(data_.stageTransitions(), id);
    return {.id = id, .at = at, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.stageTransitions(), std::move(vs));
            }};
}

Control::Rederived Control::solve(const Signal& signal, const Graph& graph) {
    Rederived out;
    for (auto id : dirty<NodeId>()) {
        if (graph.data().nodes().alive(id)) {
            recomputeDerived(id, signal);
            out.nodes.insert(id);
        } else {
            evictDerived(id);
        }
    }
    dirty<NodeId>().clear();
    return out;
}

void Control::recomputeDerived(NodeId id, const Signal& signal) {
    const auto* nc = data_.nodeControls().get(id);
    if (!nc) return;
    for (TimingId tid : nc->timings) {
        assert(data_.timings().alive(tid));
        const auto* ig = signal.derived().intergreenMatrices().get(id);
        const auto* t = data_.timings().get(tid);
        assert(ig);
        auto groupParams = compute::groupParams(signal, regulations_, id);
        const auto& steps =
            control_compute::resolveStepSequence(data_, t->params().steps);

        PlanDerived plan{
            .core = control::buildTimingPlan(steps),
            .intergreenViolations =
                control::validatePlanIntergreen(plan.core, *ig, groupParams),
            .minimumViolations =
                control::validatePlanMinimums(plan.core, groupParams),
        };

        derived_.plans().set(tid, std::move(plan));
    }
}

void Control::evictDerived(NodeId id) {
    const auto* nc = data_.nodeControls().get(id);
    if (!nc) return;
    for (auto tid : nc->timings) derived_.plans().erase(tid);
}