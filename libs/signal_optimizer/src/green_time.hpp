#pragma once
#include <optional>

#include "types.hpp"

// Distributes slack green time one second at a time, always extending the phase
// that yields the greatest marginal reduction in total delay.
// Returns nullopt if transition overhead alone exceeds T.
std::optional<GGreen> greedyMarginalAlloc(
    const std::vector<GPhase>& phases,
    const std::vector<GTransition>& transitions, const GConfig& cfg, int T);
