#pragma once
#include "types.hpp"

// Generates all valid GConfig instances by cross-producting per-entry group
// layouts (contiguous cluster partitions * protected/permitted assignments)
std::vector<GConfig> generateConfigs(const GInput& inp);
