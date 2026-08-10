#pragma once

#include <common/matrix.hpp>

#include "road_network/util/ids.hpp"

// Per-config intergreen times between signal groups (evac group → appr group).
using IntergreenMatrix = Matrix<SignalGroupId, int>;
