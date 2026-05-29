#pragma once

#include "road_network/util/ids.hpp"

using IntergreenMatrix =
    std::unordered_map<SignalGroupId, std::unordered_map<SignalGroupId, int>>;

IntergreenMatrix buildIntergreenMatrix(const Configuration& cfg,
                                       const IntergreenMap& igMap,
                                       const PermittedMap& permMap);