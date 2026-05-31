#pragma once
#include "types.hpp"

// Generates all valid GConfig instances by cross-producting per-entry group
// layouts (contiguous cluster partitions × protected/permitted assignments),
// appending crosswalk groups, and computing intergreen matrices and
// compatibility graphs for each combination.
std::vector<GConfig> generateConfigs(const GInput& inp);