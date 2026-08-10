#pragma once

#include "aspect_interval.hpp"

namespace control {

struct GroupParams {
    int minActiveTime;
    AspectInterval from;
    AspectInterval to;
};

}  // namespace control