#include "delay.hpp"

#include <cmath>
#include <limits>

#include "types.hpp"

float totalDelay(const GGreen& alloc, const GConfig& cfg, int T) {
    float D = 0.f;
    auto Tf = static_cast<float>(T);
    const auto& groups = cfg.groups;
    for (int i = 0; i < groups.size(); ++i) {
        auto tgr = static_cast<float>(alloc.tGroup[i]);
        D += groupDelay(groups[i], {.greenTime = tgr, .T = Tf});
    }
    return D;
}

float groupDelay(const GGroup& g, LambdaCtx lCtx) {
    if (g.kind == GKind::Crosswalk) return 0.f;

    float infinity = std::numeric_limits<float>::infinity();
    if (lCtx.T <= 1e-6f) return infinity;
    float lambda = lCtx.greenTime / lCtx.T;
    if (lambda < 0.f || lambda > 1.f) return infinity;

    float D = 0.f;
    for (const auto& gc : g.clusters) {
        if (gc.Q <= 0.f) continue;
        if (gc.S <= 0.f) return infinity;

        float x_c = (gc.Q / gc.S) / lambda;
        float c_c = gc.S * lambda;
        float denom = 1.f - std::min(1.f, x_c) * lambda;
        if (denom <= 1e-6) return infinity;

        float d1 = lCtx.T * (1.f - lambda) * (1.f - lambda) / (2.f * denom);
        float sqArg =
            (x_c - 1.f) * (x_c - 1.f) + (c_c > 0.f ? x_c * x_c / c_c : 0.f);
        if (sqArg < 0.f) return infinity;

        float d2 = 900.f * ((x_c - 1.f) + std::sqrt(sqArg));
        D += (d1 + d2) * gc.Q;
    }
    return D;
}