#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace control_theory {
// overflow-delay constant [s] - 15-minute (900 s) analysis period.
inline constexpr double kOverflowPeriod = 900.0;
inline constexpr double kEpsilon = 1e-6;

// Capacity / delay metrics for a single demand stream cluster under a fixed
// green allocation.
struct StreamDelay {
    double greenRatio = 0;  // λ = g+1/T         [-]
    double capacity = 0;    // c = S * λ           [veh/h]
    double saturation = 0;  // x = Q/c           [-]
    double delay = 0;       // d = d1 + d2       [s/veh];
};

[[nodiscard]] inline StreamDelay streamDelay(double flow, double saturationFlow,
                                             double effectiveGreen,
                                             double cycle) {
    constexpr double inf = std::numeric_limits<double>::infinity();

    StreamDelay r;
    if (cycle <= kEpsilon) {
        r.delay = inf;
        return r;
    }

    const double lambda = effectiveGreen / cycle;
    r.greenRatio = lambda;
    if (lambda < 0.0 || lambda > 1.0) {
        r.delay = inf;
        return r;
    }

    r.capacity = saturationFlow * lambda;
    if (flow <= 0.0) return r;  // no demand - no delay
    if (r.capacity <= 0.0) {
        r.saturation = inf;
        r.delay = inf;
        return r;
    }

    const double x = flow / r.capacity;
    r.saturation = x;

    const double denom = 1.0 - std::min(1.0, x) * lambda;
    if (denom <= kEpsilon) {
        r.delay = inf;
        return r;
    }

    const double d1 = cycle * (1.0 - lambda) * (1.0 - lambda) / (2.0 * denom);
    const double sq = (x - 1.0) * (x - 1.0) + x * x / r.capacity;
    const double d2 =
        kOverflowPeriod * ((x - 1.0) + std::sqrt(std::max(0.0, sq)));
    r.delay = d1 + d2;
    return r;
}

// Per-vehicle delay [s/veh] only.
[[nodiscard]] inline double perVehicleDelay(double flow, double saturationFlow,
                                            double effectiveGreen,
                                            double cycle) {
    return streamDelay(flow, saturationFlow, effectiveGreen, cycle).delay;
}

// Demand-weighted delay [veh * s/h] = d * Q — the optimizer's minimization
// goal.
[[nodiscard]] inline double weightedDelay(double flow, double saturationFlow,
                                          double effectiveGreen, double cycle) {
    if (flow <= 0.0) return 0.0;
    return perVehicleDelay(flow, saturationFlow, effectiveGreen, cycle) * flow;
}
}  // namespace control_theory