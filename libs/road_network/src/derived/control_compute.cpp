#include "control_compute.hpp"

#include "road_network/control/control_data.hpp"

namespace control_compute {

std::vector<control::StageStep<SignalGroupId>> resolveStepSequence(
    const ControlData& control, const std::vector<StageStep>& sequence) {
    std::vector<control::StageStep<SignalGroupId>> out;
    out.reserve(sequence.size());
    for (const auto& step : sequence) {
        const auto* s = control.stages().get(step.stage());
        const auto* t = control.stageTransitions().get(step.transition());
        assert(s && t);
        out.push_back({.stage = s->params().core,
                       .transition = t->params().core,
                       .stageDuration = step.stageDuration()});
    }
    return out;
}

}  // namespace control_compute