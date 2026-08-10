#pragma once

#include <vector>

#include "regulations/types/stream_kind.hpp"

namespace reg {

struct Intergreen {
    struct FloorRule {
        StreamKind evac;
        StreamKind appr;
        double value;
    };
    double approachReactionMargin{1.0};  // t_d = s_d / v_d + 1
    int defaultFloor{0};                 // floor for any unlisted pair
    std::vector<FloorRule> floors{};
};

}  // namespace reg