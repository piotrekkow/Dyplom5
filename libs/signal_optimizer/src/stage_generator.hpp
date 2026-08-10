#pragma once
#include "types.hpp"

// Returns all maximal cliques of the compatibility graph via Bron-Kerbosch
// with pivoting; each clique is a candidate signal stage.
std::vector<GStage> findMaximalCliques(const GConfig& cfg);
