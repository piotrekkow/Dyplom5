#include "stage_generator.hpp"

#include <algorithm>
#include <numeric>
#include <vector>
#include <iterator>

#include "types.hpp"

namespace {

bool isNeighbor(const GConfig& cfg, GId u, GId v) {
    return cfg.compatibility.at(u).contains(v);
}

auto countNeighborsInP(const GConfig& cfg, GId u, const std::vector<GId>& P) {
    return std::ranges::count_if(P,
                                 [&](GId v) { return isNeighbor(cfg, u, v); });
}

void bronKerbosch(std::vector<GId>& R, std::vector<GId> P, std::vector<GId> X,
                  const GConfig& cfg, std::vector<GStage>& result) {
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

        R.push_back(v);
        bronKerbosch(R, std::move(nextP), std::move(nextX), cfg, result);
        R.pop_back();

        std::erase(P, v);
        X.push_back(v);
    }
}
}  // namespace

std::vector<GStage> findMaximalCliques(const GConfig& cfg) {
    std::vector<GStage> result;
    std::vector<GId> P(cfg.groups.size());
    std::ranges::iota(P, 0);
    std::vector<GId> R, X;
    bronKerbosch(R, std::move(P), X, cfg, result);
    return result;
}
