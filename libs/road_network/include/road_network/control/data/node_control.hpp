#pragma once

#include <vector>

#include "road_network/util/ids.hpp"

struct NodeControl {
    std::vector<TimingId> timings;
    std::vector<StageId> stages;
    std::vector<StageTransitionId> transitions;
};