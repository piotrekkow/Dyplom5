#pragma once
#include "types.hpp"

// Total demand-weighted delay (s/h) summed across all groups in cfg at the
// given green allocation.
double totalDelay(const GGreen& alloc, const GConfig& cfg, int T);

// Demand-weighted delay (s/h) for a single group at a given green time and
// cycle length. Crosswalk groups don't factor into the vehicle-delay
// objective and always return 0.
double groupDelay(const GGroup& g, double greenTime, double cycle);
