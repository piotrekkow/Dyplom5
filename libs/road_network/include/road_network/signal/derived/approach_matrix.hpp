#pragma once

#include <common/matrix.hpp>

#include "road_network/util/ids.hpp"

// Directed approach caps between signal groups. A[i][j] = c (c >= 0) means
// group i must begin its green no later than c seconds after group j does,
// because j's traffic reaches the point they share in that time. A negative or
// absent entry means the constraint does not apply in that direction. Only
// populated for permitted pairs (those with no intergreen governing them) whose
// leader j is a vehicle group.
using ApproachMatrix = Matrix<SignalGroupId, int>;
