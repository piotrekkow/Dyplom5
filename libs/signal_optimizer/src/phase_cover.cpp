#include "phase_cover.hpp"

#include <algorithm>
#include <cassert>
#include <unordered_set>

namespace {

// Canonical form of a cover for deduplication
std::vector<std::vector<GId>> canonical(const std::vector<GPhase>& cover) {
    std::vector<std::vector<GId>> c;
    c.reserve(cover.size());
    for (const auto& ph : cover) {
        auto ids = ph.groupIds;
        std::ranges::sort(ids);
        c.push_back(std::move(ids));
    }
    std::ranges::sort(c);
    return c;
}

// Greedy cover starting from a forced first phase index
// Returns the cover or empty vector if forced phase cannot start a valid cover
std::vector<GPhase> greedyCover(const std::vector<GPhase>& validPhases,
                                const GConfig& cfg, int forcedFirstIdx) {
    // Required groups: all movement + crosswalk groups
    std::unordered_set<GId> required;
    for (const auto& g : cfg.groups) required.insert(g.id);

    std::unordered_set<GId> covered;
    std::vector<GPhase> cover;

    // Seed with forced first phase
    const auto& first = validPhases[forcedFirstIdx];
    cover.push_back(first);
    for (GId gid : first.groupIds) covered.insert(gid);

    while (covered.size() < required.size()) {
        // Find uncovered required groups
        std::unordered_set<GId> uncovered;
        for (GId gid : required) {
            if (covered.find(gid) == covered.end()) uncovered.insert(gid);
        }

        // Pick phase covering most uncovered (tie: highest sum maxY)
        int bestIdx = -1;
        int bestNew = -1;
        float bestY = -1.0;
        for (int pi = 0; pi < (int)validPhases.size(); pi++) {
            const auto& ph = validPhases[pi];
            int newCovered = 0;
            float newY = 0.0;
            for (GId gid : ph.groupIds) {
                if (uncovered.count(gid) > 0) {
                    newCovered++;
                    newY = std::max(newY, cfg.groups[gid].maxY);
                }
            }
            if (newCovered > bestNew ||
                (newCovered == bestNew && newY > bestY)) {
                bestIdx = pi;
                bestNew = newCovered;
                bestY = newY;
            }
        }

        if (bestIdx < 0 || bestNew == 0) {
            // Cannot cover remaining required groups — add any phase with
            // covered groups (shouldn't normally happen)
            assert(false);
            break;
        }

        const auto& best_ph = validPhases[bestIdx];
        cover.push_back(best_ph);
        for (GId gid : best_ph.groupIds) covered.insert(gid);
    }

    // Verify all required groups are covered
    for (GId gid : required) {
        if (covered.find(gid) == covered.end()) return {};
    }
    return cover;
}

}  // namespace

std::vector<std::vector<GPhase>> generateCovers(
    const std::vector<GPhase>& validPhases, const GConfig& cfg) {
    if (validPhases.empty()) return {};

    std::vector<std::vector<std::vector<GId>>> seen_canonicals;
    std::vector<std::vector<GPhase>> result;

    auto try_add = [&](std::vector<GPhase>&& cover) {
        if (cover.empty()) return;
        auto c = canonical(cover);
        for (const auto& prev : seen_canonicals) {
            if (prev == c) return;
        }
        seen_canonicals.push_back(c);
        result.push_back(std::move(cover));
    };

    for (int i = 0; i < (int)validPhases.size(); i++) {
        try_add(greedyCover(validPhases, cfg, i));
    }

    return result;
}
