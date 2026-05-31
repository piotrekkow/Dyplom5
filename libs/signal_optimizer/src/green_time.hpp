#pragma once
#include <optional>

#include "types.hpp"

std::optional<GGreen> greedyMarginalAlloc(
    const std::vector<GPhase>& phases,
    const std::vector<GTransition>& transitions, const GConfig& cfg, int T);

// Remove up to `deficit` seconds from phases, cheapest-first (least delay
// increase per second). Stops early if no phase can lose a second without
// violating tMin. Returns the accumulated delay increase.
float removeCheapestSeconds(GGreen& alloc, const std::vector<GPhase>& phases,
                            const GConfig& cfg, int T, int deficit);
