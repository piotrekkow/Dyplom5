#pragma once

#include <control/schema/timing_plan.hpp>

#include "control/violations/intergreen_violation.hpp"
#include "control/violations/minimum_violation.hpp"
#include "road_network/util/ids.hpp"

struct PlanDerived {
    control::TimingPlan<SignalGroupId> core;
    std::vector<control::IntergreenViolation<SignalGroupId>>
        intergreenViolations;
    std::vector<control::MinimumViolation<SignalGroupId>> minimumViolations;
};