#pragma once

#include <regulations/types/signal_aspect.hpp>

namespace control {

struct AspectInterval {
    SignalAspect aspect;
    int duration;
    bool isIntergreen;
};

}  // namespace control