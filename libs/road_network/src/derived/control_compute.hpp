#pragma once

#include <control/schema/stage_step.hpp>

#include "road_network/util/ids.hpp"

class StageStep;
class ControlData;

namespace control_compute {
std::vector<control::StageStep<SignalGroupId>> resolveStepSequence(
    const ControlData& control, const std::vector<StageStep>& sequence);
}