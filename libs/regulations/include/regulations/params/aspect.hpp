#pragma once

#include "regulations/types/signal_aspect.hpp"

namespace reg {

struct Aspect {
    struct MinGreen {
        int fixed{0};
    };
    struct Transition {
        SignalAspect aspect{SignalAspect::DARK};
        int duration{0};
        bool isIntergreen{true};
    };
    SignalAspect active{SignalAspect::GREEN};
    SignalAspect inactive{SignalAspect::RED};
    Transition toActive{};
    Transition toInactive{};
    int additionalIntergreenEvac{0};
    MinGreen minGreen{};
};

}  // namespace reg