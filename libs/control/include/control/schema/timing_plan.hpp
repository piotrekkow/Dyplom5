#pragma once

#include <common/concepts.hpp>
#include <unordered_map>
#include <vector>

#include "aspect_interval.hpp"
#include "window.hpp"

namespace control {

// timing_math.cpp
std::vector<Window> activeWindows(const std::vector<AspectInterval>& sequence);

template <ComparableKey GroupKey>
struct TimingPlan {
    std::unordered_map<GroupKey, std::vector<AspectInterval>> sequences;
    [[nodiscard]] int cycleLength() const {
        int total = 0;
        if (!sequences.empty()) {
            auto it = sequences.begin();
            for (const auto& seq : it->second) {
                total += seq.duration;
            }
        }
        return total;
    }
    [[nodiscard]] std::unordered_map<GroupKey, std::vector<Window>>
    activeWindows() const {
        std::unordered_map<GroupKey, std::vector<Window>> windows;
        for (const auto& [g, seq] : sequences)
            windows.emplace(g, control::activeWindows(seq));
        return windows;
    }
};

}  // namespace control