#include "phase_generator.hpp"

#include <algorithm>
#include <numeric>
#include <vector>

#include "types.hpp"

namespace {

// check if a vertex is a neighbor of another
bool isNeighbor(const GConfig& cfg, GId u, GId v) {
    return cfg.compatibility.at(u).contains(v);
}

// count common elements between a vertex's neighbors and set P
auto countNeighborsInP(const GConfig& cfg, GId u, const std::vector<GId>& P) {
    return std::ranges::count_if(P,
                                 [&](GId v) { return isNeighbor(cfg, u, v); });
}

void bronKerbosch(std::vector<GId>& R, std::vector<GId> P, std::vector<GId> X,
                  const GConfig& cfg, std::vector<GPhase>& result) {
    // Base Case: Maximal clique found
    if (P.empty() && X.empty()) {
        if (!R.empty()) {
            result.emplace_back(R);
        }
        return;
    }

    // 1. Pivot Selection: maximize |N(pivot) ∩ P| over (P ∪ X)
    auto maxNeighborComp = [&](GId a, GId b) {
        return countNeighborsInP(cfg, a, P) < countNeighborsInP(cfg, b, P);
    };

    GId pivot = -1;
    if (!P.empty()) pivot = *std::ranges::max_element(P, maxNeighborComp);
    if (!X.empty()) {
        GId pivotX = *std::ranges::max_element(X, maxNeighborComp);
        if (pivot == -1 || countNeighborsInP(cfg, pivotX, P) >
                               countNeighborsInP(cfg, pivot, P)) {
            pivot = pivotX;
        }
    }

    // 2. Filter Candidates: Loop over v ∈ P \ N(pivot)
    std::vector<GId> candidates;
    std::ranges::copy_if(P, std::back_inserter(candidates), [&](GId v) {
        return pivot == -1 || !isNeighbor(cfg, pivot, v);
    });

    // 3. Iterative Exploration
    for (GId v : candidates) {
        std::vector<GId> nextP, nextX;

        // Form nextP = P ∩ N(v) and nextX = X ∩ N(v)
        std::ranges::copy_if(P, std::back_inserter(nextP),
                             [&](GId u) { return isNeighbor(cfg, v, u); });
        std::ranges::copy_if(X, std::back_inserter(nextX),
                             [&](GId u) { return isNeighbor(cfg, v, u); });

        // Explore
        R.push_back(v);
        bronKerbosch(R, std::move(nextP), std::move(nextX), cfg, result);
        R.pop_back();

        // Move v from P to X
        std::erase(P, v);
        X.push_back(v);
    }
}
}  // namespace

std::vector<GPhase> findMaximalCliques(const GConfig& cfg) {
    std::vector<GPhase> result;
    std::vector<GId> P(cfg.groups.size());
    std::ranges::iota(P, 0);
    std::vector<GId> R, X;
    bronKerbosch(R, std::move(P), X, cfg, result);
    return result;
}

namespace {

void enumerateCliques(const GConfig& cfg, const std::vector<GId>& candidates,
                      std::vector<GId>& current, std::vector<GPhase>& result) {
    result.emplace_back(current);
    for (size_t i = 0; i < candidates.size(); ++i) {
        GId v = candidates[i];
        std::vector<GId> nextCandidates;
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (isNeighbor(cfg, v, candidates[j])) {
                nextCandidates.push_back(candidates[j]);
            }
        }
        current.push_back(v);
        enumerateCliques(cfg, nextCandidates, current, result);
        current.pop_back();
    }
}

}  // namespace

// Enumerates every valid phase (all cliques, not just maximal ones).
// Drop-in swap for findMaximalCliques to test full search space tractability.
std::vector<GPhase> findAllCliques(const GConfig& cfg) {
    auto N = static_cast<GId>(cfg.groups.size());
    std::vector<GPhase> result;
    for (GId v = 0; v < N; ++v) {
        std::vector<GId> candidates;
        for (GId u = v + 1; u < N; ++u) {
            if (isNeighbor(cfg, v, u)) {
                candidates.push_back(u);
            }
        }
        std::vector<GId> current = {v};
        enumerateCliques(cfg, candidates, current, result);
    }
    return result;
}