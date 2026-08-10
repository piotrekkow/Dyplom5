#pragma once

#include <vector>

#include "stage_step.hpp"

namespace control {

template <ComparableKey GroupKey>
using StageSequence = std::vector<StageStep<GroupKey>>;

}