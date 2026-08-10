#pragma once
#include "types.hpp"

// Generates distinct stage covers (sets of stages that collectively serve every
// group) via greedy construction seeded from each valid stage in turn.
std::vector<std::vector<GStage>> generateCovers(
    const std::vector<GStage>& valid_stages, const GConfig& cfg);
