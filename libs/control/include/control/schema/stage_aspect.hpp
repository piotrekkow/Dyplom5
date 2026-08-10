#pragma once

#include "regulations/types/signal_aspect.hpp"

namespace control {

struct StageAspect {
    SignalAspect aspect;
    bool isActive;
};

};  // namespace control