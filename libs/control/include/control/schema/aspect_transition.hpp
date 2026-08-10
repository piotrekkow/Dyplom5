#pragma once

#include <vector>

#include "aspect_interval.hpp"

namespace control {

struct AspectTransition {
    std::vector<AspectInterval> intervals;
    [[nodiscard]] int duration() const {
        int total = 0;
        for (const auto& i : intervals) total += i.duration;
        return total;
    }
};

}  // namespace control