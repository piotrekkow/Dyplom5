#include "road_network/demand/data/flow.hpp"

#include <algorithm>

void Flow::setVehiclesPerHour(float value) {
    vehiclesPerHour_ = std::max(value, 0.f);
}

void Flow::setUc(float value) { uc_ = std::clamp(value, 0.f, 1.f); }
