#pragma once
#include "types.hpp"

struct LambdaCtx {
    float greenTime;
    float T;
};

float totalDelay(const GGreen& alloc, const GConfig& cfg, int T);
float groupDelay(const GGroup& g, LambdaCtx lCtx);