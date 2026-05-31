#pragma once
#include "types.hpp"

struct LambdaCtx {
    float greenTime;
    float T;
};

// MOP-SZS-04 total delay (s/h) summed across all movement groups in cfg.
float totalDelay(const GGreen& alloc, const GConfig& cfg, int T);

// MOP-SZS-04 delay (s/veh) for a single group given λ = greenTime/T.
// Returns infinity on oversaturation or degenerate inputs (zero saturation,
// T≤0).
float groupDelay(const GGroup& g, LambdaCtx lCtx);