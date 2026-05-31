#include "green_time.hpp"

#include <algorithm>
#include <cassert>
#include <numeric>

#include "delay.hpp"
#include "types.hpp"

namespace {
int findPhaseMin(const GPhase& phase, const GTransition& next,
                 const GConfig& cfg) {
    int min = 0;
    for (auto gid : phase.groupIds) {
        min = std::max(min, cfg.groups[gid].tMin - next.groups.at(gid).green);
    }
    return min;
}
}  // namespace

std::optional<GGreen> greedyMarginalAlloc(
    const std::vector<GPhase>& phases,
    const std::vector<GTransition>& transitions, const GConfig& cfg, int T) {
    int L = 0;
    for (const auto& tr : transitions) L += tr.makespan;
    if (T - L < 0) return std::nullopt;

    std::vector<int> fixedGreen(cfg.groups.size(), 0);
    for (const auto& tr : transitions)
        for (const auto& [gid, gtr] : tr.groups) fixedGreen[gid] += gtr.green;

    GGreen alloc{
        .tPhase = std::vector<int>(phases.size(), 0),
        .tGroup = fixedGreen,
    };

    for (int i = 0; i < phases.size(); ++i) {
        for (GId gid : phases[i].groupIds) {
            alloc.tPhase[i] = std::max(alloc.tPhase[i],
                                       cfg.groups[gid].tMin - fixedGreen[gid]);
        }
    }

    for (int i = 0; i < (int)phases.size(); ++i)
        for (GId gid : phases[i].groupIds) alloc.tGroup[gid] += alloc.tPhase[i];

    int minSum = std::reduce(alloc.tPhase.begin(), alloc.tPhase.end());
    int slack = T - L - minSum;
    if (slack < 0) return std::nullopt;

    auto Tf = static_cast<float>(T);
    auto reduction = [&](int phaseIdx) {
        float delta = 0.f;
        for (GId gid : phases[phaseIdx].groupIds) {
            auto gtf = static_cast<float>(alloc.tGroup[gid]);
            const auto& g = cfg.groups[gid];
            delta += groupDelay(g, {.greenTime = gtf, .T = Tf}) -
                     groupDelay(g, {.greenTime = gtf + 1, .T = Tf});
        }
        return delta;
    };

    std::vector<int> xwalkCounts(phases.size(), 0);
    for (int i = 0; i < (int)phases.size(); ++i)
        for (GId gid : phases[i].groupIds)
            if (cfg.groups[gid].kind == GKind::Crosswalk) xwalkCounts[i]++;

    while (slack-- > 0) {
        int bestIdx = 0;
        float bestDelta = reduction(0);
        int bestXwalks = xwalkCounts[0];

        for (int i = 1; i < (int)phases.size(); ++i) {
            float delta = reduction(i);
            if (delta > bestDelta ||
                (delta == bestDelta && xwalkCounts[i] > bestXwalks)) {
                bestDelta = delta;
                bestIdx = i;
                bestXwalks = xwalkCounts[i];
            }
        }

        alloc.tPhase[bestIdx]++;
        for (GId gid : phases[bestIdx].groupIds) alloc.tGroup[gid]++;
    }

    return std::move(alloc);
}