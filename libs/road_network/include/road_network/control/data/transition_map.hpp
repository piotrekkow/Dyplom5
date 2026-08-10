#pragma once
#include <unordered_map>

#include "road_network/util/ids.hpp"

using TransitionMap =
    std::unordered_map<StageId, std::unordered_map<StageId, StageTransitionId>>;