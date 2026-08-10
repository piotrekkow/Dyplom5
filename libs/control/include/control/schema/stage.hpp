#pragma once

#include <common/concepts.hpp>
#include <unordered_map>

#include "stage_aspect.hpp"

namespace control {

template <ComparableKey GroupKey>
struct Stage {
    std::unordered_map<GroupKey, StageAspect> groups;
};

}  // namespace control