#pragma once
#include "types.hpp"

// Returns all maximal cliques of the compatibility graph via Bron-Kerbosch
// with pivoting; each clique is a candidate signal phase.
std::vector<GPhase> findMaximalCliques(const GConfig& cfg);

// Returns all cliques, including non-maximal ones. Used by the optimizer
// (instead of maximal-only) so that every valid phase subset is explored.
std::vector<GPhase> findAllCliques(const GConfig& cfg);
