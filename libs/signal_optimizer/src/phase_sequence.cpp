#include "phase_sequence.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_set>

#include "types.hpp"

namespace {
    int effClearance(const GConfig& cfg, GId ev, GId ap) {
        int ig = cfg.intergreen[ev][ap];
        return cfg.groups[ev].kind == GKind::Crosswalk ? std::max(ig + 4, 4) : std::max(ig, 3);
    }
}

GTransition computeTransitionSimple(const GPhase& from, const GPhase& to,
                                    const GConfig& cfg) {
    std::unordered_set<GId> fromSet(from.groupIds.begin(), from.groupIds.end());
    std::unordered_set<GId> toSet(to.groupIds.begin(), to.groupIds.end());

    std::vector<GId> fromOnly, toOnly, bothOnly;
    for (GId gid : from.groupIds)
        (toSet.count(gid) ? bothOnly : fromOnly).push_back(gid);
    for (GId gid : to.groupIds)
        if (!fromSet.count(gid)) toOnly.push_back(gid);

    // Approach minimum (RedYellow)
    int makespan = 0;
    for (GId gid : toOnly)
        if (cfg.groups[gid].kind != GKind::Crosswalk)
            makespan = 1;  // reserve 1s for redyellow transition

    std::unordered_map<GId, int> igMax;
    for (GId g_ev : fromOnly) {
        int ig = 0;
        for (GId g_ap : toOnly) ig = std::max(ig, cfg.intergreen[g_ev][g_ap]);
        bool xwalk = cfg.groups[g_ev].kind == GKind::Crosswalk;
        igMax[g_ev] = xwalk ? std::max(ig + 4, 4) : std::max(ig, 3);
        makespan = std::max(makespan, igMax[g_ev]);
    }

    GTransition tr;
    tr.makespan = makespan;
    for (GId gid : bothOnly)
        tr.groups[gid] = {.green = makespan,
                          .kind = cfg.groups[gid].kind,
                          .prevActive = true,
                          .nextActive = true};
    for (GId gid : fromOnly)
        tr.groups[gid] = {.green = makespan - igMax[gid],
                          .kind = cfg.groups[gid].kind,
                          .prevActive = true,
                          .nextActive = false};
    for (GId gid : toOnly)
        tr.groups[gid] = {.green = 0,
                          .kind = cfg.groups[gid].kind,
                          .prevActive = false,
                          .nextActive = true};

    return tr;
}

GTransition computeTransitionWeighted(const GPhase& from, const GPhase& to,
                                      const GConfig& cfg) {
    std::unordered_set<GId> fromSet(from.groupIds.begin(), from.groupIds.end());
    std::unordered_set<GId> toSet(to.groupIds.begin(), to.groupIds.end());

    std::vector<GId> fromOnly, toOnly, bothOnly;
    for (GId gid : from.groupIds)
        (toSet.count(gid) ? bothOnly : fromOnly).push_back(gid);
    for (GId gid : to.groupIds)
        if (!fromSet.count(gid)) toOnly.push_back(gid);

    // Makespan — identical to V1
    int makespan = 0;
    for (GId gid : toOnly)
        if (cfg.groups[gid].kind != GKind::Crosswalk) makespan = 1;

    for (GId g_ev : fromOnly) {
        int ig = 0;
        for (GId g_ap : toOnly) ig = std::max(ig, cfg.intergreen[g_ev][g_ap]);
        bool xwalk = cfg.groups[g_ev].kind == GKind::Crosswalk;
        int igM = xwalk ? std::max(ig + 4, 4) : std::max(ig, 3);
        makespan = std::max(makespan, igM);
    }

    std::unordered_map<GId, int> greenEnd;
    std::unordered_map<GId, int> startTime;
    for (GId gid : fromOnly) greenEnd[gid] = 0;
    for (GId gid : toOnly) startTime[gid] = makespan;

    // Process all groups in descending maxY order. Higher-maxY groups are
    // committed first; lower-maxY groups yield to already-committed values.
    std::vector<GId> allGroups;
    allGroups.insert(allGroups.end(), fromOnly.begin(), fromOnly.end());
    allGroups.insert(allGroups.end(), toOnly.begin(), toOnly.end());
    std::ranges::sort(allGroups, [&](GId a, GId b) {
        return cfg.groups[a].maxY > cfg.groups[b].maxY;
    });

    std::unordered_set<GId> processed;

    for (GId g : allGroups) {
        if (!toSet.count(g)) {
            // fromOnly: push greenEnd[g] as high as constraints allow.
            // Already-committed (higher-maxY) toOnly groups cap greenEnd via
            // their locked startTime; not-yet-committed groups are assumed to
            // start no later than makespan (they may sacrifice green for g).
            int ceiling = makespan;
            for (GId t : toOnly) {
                int bound = (processed.count(t) ? startTime[t] : makespan) -
                            effClearance(cfg, g, t);
                ceiling = std::min(ceiling, bound);
            }
            greenEnd[g] = std::max(0, ceiling);
        } else {
            // toOnly: pull startTime[g] as low as possible using current
            // greenEnd values. Unprocessed fromOnly groups contribute 0 (their
            // minimum), but when they are later committed they are capped by
            // this startTime via the processed-set ceiling above.
            int floor = 0;
            for (GId f : fromOnly)
                floor = std::max(floor, greenEnd[f] + effClearance(cfg, f, g));
            startTime[g] = floor;
        }
        processed.insert(g);
    }

    GTransition tr;
    tr.makespan = makespan;
    for (GId gid : bothOnly)
        tr.groups[gid] = {.green = makespan,
                          .kind = cfg.groups[gid].kind,
                          .prevActive = true,
                          .nextActive = true};
    for (GId gid : fromOnly)
        tr.groups[gid] = {.green = greenEnd[gid],
                          .kind = cfg.groups[gid].kind,
                          .prevActive = true,
                          .nextActive = false};
    for (GId gid : toOnly)
        tr.groups[gid] = {.green = makespan - startTime[gid],
                          .kind = cfg.groups[gid].kind,
                          .prevActive = false,
                          .nextActive = true};

    return tr;
}

std::vector<std::pair<std::vector<int>, std::vector<GTransition>>>
enumerateSequences(const std::vector<GPhase>& cover, const GConfig& cfg) {
    int k = static_cast<int>(cover.size());
    if (k == 0) return {};

    std::vector<std::pair<std::vector<int>, std::vector<GTransition>>> result;

    // Fix phase at index 0, permute the rest
    std::vector<int> perm(k - 1);
    std::ranges::iota(perm, 1);

    do {
        std::vector<int> order = {0};
        order.insert(order.end(), perm.begin(), perm.end());

        std::vector<GTransition> transitions(k);
        for (int i = 0; i < k; i++) {
            const GPhase& from = cover[order[i]];
            const GPhase& to = cover[order[(i + 1) % k]];
            transitions[i] = computeTransitionWeighted(from, to, cfg);
        }
        result.emplace_back(order, std::move(transitions));
    } while (std::next_permutation(perm.begin(), perm.end()));

    return result;
}
