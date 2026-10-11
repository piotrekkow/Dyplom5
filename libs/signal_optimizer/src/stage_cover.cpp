#include "stage_cover.hpp"

#include <algorithm>
#include <cassert>
#include <unordered_set>

namespace {

// deduplication
std::vector<std::vector<GId>> canonical(const std::vector<GStage>& cover) {
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

// Greedy cover starting from a forced first stage index
// Returns the cover or empty vector if forced stage cannot start a valid cover
std::vector<GStage> greedyCover(const std::vector<GStage>& validStages,
                                const GConfig& cfg, int forcedFirstIdx) {
    // Required groups: all movement + crosswalk groups
    std::unordered_set<GId> required;
    for (const auto& g : cfg.groups) required.insert(g.id);

    std::unordered_set<GId> covered;
    std::vector<GStage> cover;

    // Seed with forced first stage
    const auto& first = validStages[static_cast<std::size_t>(forcedFirstIdx)];
    cover.push_back(first);
    for (GId gid : first.groupIds) covered.insert(gid);

    while (covered.size() < required.size()) {
        // Find uncovered required groups
        std::unordered_set<GId> uncovered;
        for (GId gid : required) {
            if (covered.find(gid) == covered.end()) uncovered.insert(gid);
        }

        // Pick stage covering most uncovered (tie: highest sum maxY)
        int bestIdx = -1;
        int bestNew = -1;
        double bestY = -1.0;
        for (int pi = 0; pi < static_cast<int>(validStages.size()); pi++) {
            const auto& ph = validStages[static_cast<std::size_t>(pi)];
            int newCovered = 0;
            double newY = 0.0;
            for (GId gid : ph.groupIds) {
                if (uncovered.count(gid) > 0) {
                    newCovered++;
                    newY = std::max(
                        newY, cfg.groups[static_cast<std::size_t>(gid)].maxY);
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
            // Cannot cover remaining required groups (shouldn't normally
            // happen)
            assert(false);
            break;
        }

        const auto& best_ph = validStages[static_cast<std::size_t>(bestIdx)];
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

std::vector<std::vector<GStage>> generateCovers(
    const std::vector<GStage>& validStages, const GConfig& cfg) {
    if (validStages.empty()) return {};

    std::vector<std::vector<std::vector<GId>>> seen_canonicals;
    std::vector<std::vector<GStage>> result;

    auto try_add = [&](std::vector<GStage>&& cover) {
        if (cover.empty()) return;
        auto c = canonical(cover);
        for (const auto& prev : seen_canonicals) {
            if (prev == c) return;
        }
        seen_canonicals.push_back(c);
        result.push_back(std::move(cover));
    };

    for (int i = 0; i < static_cast<int>(validStages.size()); i++) {
        try_add(greedyCover(validStages, cfg, i));
    }

    return result;
}
