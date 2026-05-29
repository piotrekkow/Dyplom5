#pragma once

#include "road_network/util/geometry/position.hpp"

struct ConflictPoint {
    Position position;
    float evacDistance;
    float approachDistance;
};