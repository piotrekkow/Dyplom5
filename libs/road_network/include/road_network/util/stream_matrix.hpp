#pragma once

#include <common/matrix.hpp>

#include "road_network/util/ids.hpp"

// A matrix over stream pairs (evac, appr). The diagonal is always empty: a
// stream never conflicts with itself. Stream Matrix is structurally
// symmetrical.
template <typename T>
using StreamMatrix = Matrix<StreamId, T>;
