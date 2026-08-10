#include "green_time.hpp"

#include <algorithm>
#include <cassert>
#include <numeric>

#include "delay.hpp"
#include "types.hpp"

namespace {
// Sum of the non-intergreen (slack green) duration a group is granted
// during a transition — e.g. a both-active group's full makespan
int bonusGreen(const control::AspectTransition& tr) {
    int green = 0;
    for (const auto& iv : tr.intervals)
        if (!iv.isIntergreen) green += iv.duration;
    return green;
}

std::vector<GId> activeGroupIds(const control::Stage<GId>& stage) {
    std::vector<GId> ids;
    for (const auto& [gid, aspect] : stage.groups)
        if (aspect.isActive) ids.push_back(gid);
    return ids;
}
}  // namespace

std::optional<GGreen> greedyMarginalAlloc(
    const std::vector<control::Stage<GId>>& stages,
    const std::vector<control::StageTransition<GId>>& transitions,
    const GConfig& cfg, int T) {
    int L = 0;
    for (const auto& tr : transitions) L += tr.makespan();
    if (T - L < 0) return std::nullopt;

    std::vector<std::vector<GId>> active(stages.size());
    for (std::size_t i = 0; i < stages.size(); ++i)
        active[i] = activeGroupIds(stages[i]);

    std::vector<int> fixedGreen(cfg.groups.size(), 0);
    for (const auto& tr : transitions)
        for (const auto& [gid, gtr] : tr.transitions)
            fixedGreen[static_cast<std::size_t>(gid)] += bonusGreen(gtr);

    GGreen alloc{
        .tStage = std::vector<int>(stages.size(), 0),
        .tGroup = fixedGreen,
    };

    for (std::size_t i = 0; i < stages.size(); ++i) {
        for (GId gid : active[i]) {
            alloc.tStage[i] =
                std::max(alloc.tStage[i],
                         cfg.groups[static_cast<std::size_t>(gid)].tMin -
                             fixedGreen[static_cast<std::size_t>(gid)]);
        }
    }

    for (std::size_t i = 0; i < stages.size(); ++i)
        for (GId gid : active[i])
            alloc.tGroup[static_cast<std::size_t>(gid)] += alloc.tStage[i];

    int minSum = std::reduce(alloc.tStage.begin(), alloc.tStage.end());
    int slack = T - L - minSum;
    if (slack < 0) return std::nullopt;

    auto Td = static_cast<double>(T);
    auto reduction = [&](std::size_t idx) {
        double delta = 0.0;
        for (GId gid : active[idx]) {
            auto gtd = static_cast<double>(
                alloc.tGroup[static_cast<std::size_t>(gid)]);
            const auto& g = cfg.groups[static_cast<std::size_t>(gid)];
            delta += groupDelay(g, gtd, Td) - groupDelay(g, gtd + 1, Td);
        }
        return delta;
    };

    std::vector<int> xwalkCounts(stages.size(), 0);
    for (std::size_t i = 0; i < stages.size(); ++i)
        for (GId gid : active[i])
            if (cfg.groups[static_cast<std::size_t>(gid)].kind ==
                GKind::Crosswalk)
                xwalkCounts[i]++;

    while (slack-- > 0) {
        std::size_t bestIdx = 0;
        double bestDelta = reduction(0);
        int bestXwalks = xwalkCounts[0];

        for (std::size_t i = 1; i < stages.size(); ++i) {
            double delta = reduction(i);
            if (delta > bestDelta ||
                (delta == bestDelta && xwalkCounts[i] > bestXwalks)) {
                bestDelta = delta;
                bestIdx = i;
                bestXwalks = xwalkCounts[i];
            }
        }

        alloc.tStage[bestIdx]++;
        for (GId gid : active[bestIdx])
            alloc.tGroup[static_cast<std::size_t>(gid)]++;
    }

    return alloc;
}
