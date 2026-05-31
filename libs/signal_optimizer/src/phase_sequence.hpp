#pragma once
#include <utility>

#include "types.hpp"

// Evacuation clearance for a group: crosswalk adds 4 s flashing to intergreen.
int evacuationClearance(GId gid, int maxIg, const GConfig& cfg);

GTransition computeTransition(const GPhase& from, const GPhase& to,
                              const GConfig& cfg);

GTransition computeTransitionV2(const GPhase& from, const GPhase& to,
                                const GConfig& cfg);

// Returns vector of (phase_ordering_indices, transitions) for all (k-1)!
// circular orderings of the given cover.
std::vector<std::pair<std::vector<int>, std::vector<GTransition>>>
enumerateSequences(const std::vector<GPhase>& cover, const GConfig& cfg);
