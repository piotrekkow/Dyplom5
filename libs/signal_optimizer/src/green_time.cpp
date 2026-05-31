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
        for (GId gid : phases[i].groupIds)
            alloc.tGroup[gid] += alloc.tPhase[i];

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
        for (GId gid : phases[bestIdx].groupIds)
            alloc.tGroup[gid]++;
    }

    return std::move(alloc);
}

float removeCheapestSeconds(GGreen& alloc, const std::vector<GPhase>& phases,
                            const GConfig& cfg, int T, int deficit) {
    float totalPenalty = 0.f;
    auto Tf = static_cast<float>(T);

    // Delay increase from removing 1 s from phase i
    auto penalty = [&](int i) {
        float delta = 0.f;
        for (GId gid : phases[i].groupIds) {
            auto gtf = static_cast<float>(alloc.tGroup[gid]);
            const auto& g = cfg.groups[gid];
            delta += groupDelay(g, {.greenTime = gtf - 1, .T = Tf}) -
                     groupDelay(g, {.greenTime = gtf, .T = Tf});
        }
        return delta;
    };

    // Phase is removable if no group would drop below tMin
    auto canRemove = [&](int i) {
        if (alloc.tPhase[i] <= 0) return false;
        for (GId gid : phases[i].groupIds)
            if (alloc.tGroup[gid] - 1 < cfg.groups[gid].tMin) return false;
        return true;
    };

    for (int d = 0; d < deficit; ++d) {
        int bestIdx = -1;
        float bestPenalty = std::numeric_limits<float>::infinity();
        for (int i = 0; i < (int)phases.size(); ++i) {
            if (!canRemove(i)) continue;
            float p = penalty(i);
            if (p < bestPenalty) { bestPenalty = p; bestIdx = i; }
        }
        if (bestIdx < 0) break;
        alloc.tPhase[bestIdx]--;
        for (GId gid : phases[bestIdx].groupIds) alloc.tGroup[gid]--;
        totalPenalty += bestPenalty;
    }
    return totalPenalty;
}

// std::optional<GAlloc> allocateGreen(const std::vector<GPhase>& phases,
//                                     const std::vector<GTransition>&
//                                     transitions, const GConfig& cfg, int T) {
//     int L = 0;
//     for (const auto& tr : transitions) L += tr.makespan;
//     int T_avail = T - L;
//     if (T_avail < 0) return std::nullopt;

//     int k = static_cast<int>(phases.size());
//     int kMinTotal = 0;

//     auto findPhaseMin = [&](int phaseIdx) {
//         int min = 0;
//         const auto& nextTr = transitions[phaseIdx];
//         for (auto gid : phases[phaseIdx].groupIds) {
//             const auto& gCfg = cfg.groups[gid];
//             const auto& gTr = nextTr.groups.at(gid);
//             min = std::max(min, gCfg.tMin - gTr.green);
//         }
//         return min;
//     };
//     std::vector<int> phaseMins;
//     phaseMins.reserve(phases.size());
//     for (int i = 0; i < phases.size(); ++i) {
//         int iMin = findPhaseMin(i);
//         kMinTotal += iMin;
//         phaseMins.push_back(iMin);
//     }
//     if (T_avail < kMinTotal) return std::nullopt;

//     std::vector<int> tPhase(k, 0);
//     std::vector<bool> clamped(k, false);

//     // Proportional allocation with clamping loop (at most k iterations)
//     int remaining = T_avail;
//     double Y_unc = 0.0;
//     for (const auto& ph : phases) Y_unc += ph.y_star;

//     for (int iter = 0; iter < k; iter++) {
//         bool new_clamp = false;

//         if (Y_unc > 0.0) {
//             for (int i = 0; i < k; i++) {
//                 if (!clamped[i])
//                     tPhase[i] =
//                         static_cast<int>(static_cast<double>(remaining) *
//                                          phases[i].y_star / Y_unc);
//             }
//         } else {
//             // No y_star for unclamped phases → equal share
//             int n_unc = 0;
//             for (int i = 0; i < k; i++) {
//                 if (!clamped[i]) n_unc++;
//             }
//             if (n_unc > 0) {
//                 int share = remaining / n_unc;
//                 for (int i = 0; i < k; i++) {
//                     if (!clamped[i]) tPhase[i] = share;
//                 }
//             }
//         }

//         for (int i = 0; i < k; i++) {
//             if (clamped[i]) continue;
//             if (tPhase[i] < phaseMins[i]) {
//                 tPhase[i] = phaseMins[i];
//                 clamped[i] = true;
//                 remaining -= phaseMins[i];
//                 Y_unc -= phases[i].y_star;
//                 new_clamp = true;
//             }
//         }

//         if (!new_clamp) break;
//     }

//     // Absorb any integer-rounding surplus into the highest-y_star phase
//     int alloc_sum = 0;
//     for (int i = 0; i < k; i++) alloc_sum += tPhase[i];
//     int surplus = T_avail - alloc_sum;
//     if (surplus != 0) {
//         int best = 0;
//         for (int i = 1; i < k; i++) {
//             if (phases[i].y_star > phases[best].y_star) best = i;
//         }
//         tPhase[best] += surplus;
//     }

//     GAlloc alloc;
//     alloc.t_phase = tPhase;
//     alloc.onset_phase = std::vector<int>(k);
//     int t_cnt = 0;
//     for (int i = 0; i < k; i++) {
//         alloc.onset_phase[i] = t_cnt;
//         t_cnt += tPhase[i] + transitions[i].makespan;
//     }
//     assert(t_cnt == T);

//     // Initialise per-group counters
//     for (const auto& g : cfg.groups) alloc.t_gg[g.id] = 0;

//     for (int i = 0; i < k; i++) {
//         for (GId gid : phases[i].groupIds) {
//             // Phase green contribution
//             alloc.t_gg[gid] += tPhase[i];
//             alloc.phasesActive[gid].push_back(i);
//         }
//     }

//     // Transition green contribution
//     for (const auto& tr : transitions) {
//         for (const auto& [gid, gtr] : tr.groups) alloc.t_gg[gid] +=
//         gtr.green;
//     }

//     // Effective green and saturation factor
//     // for (const auto& g : cfg.groups) {
//     //     alloc.t_eg[g.id] = alloc.t_gg[g.id] + 1;
//     //     if (g.kind != GKind::Crosswalk && g.y_g > 0.0 && alloc.t_eg[g.id]
//     >
//     //     0) {
//     //         alloc.x_g[g.id] = g.y_g * static_cast<double>(T) /
//     //                           static_cast<double>(alloc.t_eg[g.id]);
//     //     } else {
//     //         alloc.x_g[g.id] = 0.0;
//     //     }
//     // }

//     return alloc;
// }
