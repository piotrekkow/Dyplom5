#pragma once
#include <utility>

#include "types.hpp"

// Compute transitions by locking all the groups in 'to' phase to start at
// makespan (last second of transition) and groups in 'from' to finish as late
// as possible while respecting intergreen
GTransition computeTransitionSimple(const GPhase& from, const GPhase& to,
                                    const GConfig& cfg);

// Compute transitions by sorting groups by the load factor y, so that most
// constrained groups gain more green in the transition
GTransition computeTransitionWeighted(const GPhase& from, const GPhase& to,
                                      const GConfig& cfg);

// Returns vector of (phase_ordering_indices, transitions) for all (k-1)!
// circular orderings of the given cover.
std::vector<std::pair<std::vector<int>, std::vector<GTransition>>>
enumerateSequences(const std::vector<GPhase>& cover, const GConfig& cfg);
