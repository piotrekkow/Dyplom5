#pragma once
#include "types.hpp"

std::vector<std::vector<GPhase>> generateCovers(
    const std::vector<GPhase>& valid_phases, const GConfig& cfg);
