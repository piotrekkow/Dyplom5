#pragma once
#include "types.hpp"

// Generates distinct phase covers (sets of phases that collectively serve every
// group) via greedy construction seeded from each valid phase in turn.
std::vector<std::vector<GPhase>> generateCovers(
    const std::vector<GPhase>& valid_phases, const GConfig& cfg);
