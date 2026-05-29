#pragma once
#include <optional>

#include "types.hpp"

std::optional<GGreen> greedyMarginalAlloc(
    const std::vector<GPhase>& phases,
    const std::vector<GTransition>& transitions, const GConfig& cfg, int T);
