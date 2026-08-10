#pragma once

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/matrix.hpp"
#include "control/schema/group_params.hpp"
#include "control/schema/stage.hpp"
#include "control/schema/stage_transition.hpp"

namespace control {

namespace detail {
template <ComparableKey GroupKey>
std::unordered_set<GroupKey> activeGroups(const Stage<GroupKey>& s) {
    std::unordered_set<GroupKey> active;
    for (const auto& [gid, aspect] : s.groups)
        if (aspect.isActive) active.insert(gid);
    return active;
}
}  // namespace detail

// Computes the weighted stage transition between `from` and `to`: for each
// group, the sequence of AspectIntervals it goes through during the
// transition. Every group present in `from`/`to` gets an entry whose
// intervals sum to the transition's makespan, including groups that stay
// inactive throughout - StageTransition::makespan() reads the makespan back
// off an arbitrary entry, so every entry must agree on the total duration.
template <ComparableKey GroupKey>
StageTransition<GroupKey> computeTransitionWeighted(
    const Stage<GroupKey>& from, const Stage<GroupKey>& to,
    const std::unordered_map<GroupKey, GroupParams>& params,
    const Matrix<GroupKey, int>& intergreen,
    const std::unordered_map<GroupKey, double>& weights,
    const Matrix<GroupKey, int>& approach = {}, bool enforceApproach = false) {
    const std::unordered_set<GroupKey> fromActive = detail::activeGroups(from);
    const std::unordered_set<GroupKey> toActive = detail::activeGroups(to);

    std::vector<GroupKey> fromOnly, toOnly, bothOnly, neither;
    for (const auto& [gid, _] : from.groups) {
        const bool inFrom = fromActive.contains(gid);
        const bool inTo = toActive.contains(gid);
        if (inFrom && inTo)
            bothOnly.push_back(gid);
        else if (inFrom)
            fromOnly.push_back(gid);
        else if (inTo)
            toOnly.push_back(gid);
        else
            neither.push_back(gid);
    }

    int makespan = 0;
    std::unordered_map<GroupKey, std::unordered_map<GroupKey, int>>
        localMakespans;

    {  // find makespan
        // toOnly's own startup stage alone lower-bounds the makespan.
        for (auto gid : toOnly)
            makespan = std::max(makespan, params.at(gid).from.duration);

        for (auto fid : fromOnly) {
            // fromOnly's own clearance stage (evacuating).
            int addFrom = 0;
            if (!params.at(fid).to.isIntergreen)
                addFrom = params.at(fid).to.duration;

            for (auto tid : toOnly) {
                // toOnly's own startup stage (approaching).
                int addTo = 0;
                if (!params.at(tid).from.isIntergreen)
                    addTo = params.at(tid).from.duration;

                int igVal = 0;
                if (auto ig = intergreen.find(fid, tid)) igVal = *ig;

                int localMakespan = std::max(addFrom + addTo + igVal,
                                             params.at(fid).to.duration);
                localMakespans[fid][tid] = localMakespan;
                makespan = std::max(makespan, localMakespan);
            }
        }
    }  // find makespan

    std::unordered_map<GroupKey, int> greenEnd;
    std::unordered_map<GroupKey, int> startTime;
    for (auto gid : fromOnly) greenEnd[gid] = 0;
    for (auto gid : toOnly) startTime[gid] = makespan;

    std::vector<GroupKey> sorted;
    sorted.insert(sorted.end(), fromOnly.begin(), fromOnly.end());
    sorted.insert(sorted.end(), toOnly.begin(), toOnly.end());
    std::ranges::sort(
        sorted, [&](auto a, auto b) { return weights.at(a) > weights.at(b); });

    std::unordered_set<GroupKey> processed;
    for (auto g : sorted) {
        if (!toActive.contains(g)) {
            int ceiling = makespan;
            for (auto t : toOnly) {
                int bound = (processed.contains(t) ? startTime[t] : makespan) -
                            localMakespans.at(g).at(t);
                ceiling = std::min(ceiling, bound);
            }
            greenEnd[g] = std::max(0, ceiling);
        } else {
            int floor = 0;
            for (auto f : fromOnly)
                floor =
                    std::max(floor, greenEnd[f] + localMakespans.at(f).at(g));
            startTime[g] = floor;
        }
        processed.insert(g);
    }

    if (enforceApproach) {
        std::vector<GroupKey> leaders = toOnly;
        leaders.insert(leaders.end(), bothOnly.begin(), bothOnly.end());
        struct Cap {
            GroupKey appr, evac;
            int cap;
        };
        std::vector<Cap> caps;
        for (auto appr : toOnly)
            for (auto evac : leaders) {
                if (appr == evac) continue;
                if (!startTime.contains(evac)) continue;  // green throughout
                if (const auto* c = approach.find(appr, evac); c && *c >= 0)
                    caps.push_back({appr, evac, *c});
            }
        for (bool changed = true; changed;) {
            changed = false;
            for (const auto& c : caps) {
                const int need = std::min(startTime[c.appr] - c.cap, makespan);
                if (need > startTime[c.evac]) {
                    startTime[c.evac] = need;
                    changed = true;
                }
            }
        }
    }

    StageTransition<GroupKey> tr;
    for (auto gid : bothOnly) {
        auto& ivs = tr.transitions[gid].intervals;
        if (makespan > 0)
            ivs.push_back({.aspect = from.groups.at(gid).aspect,
                           .duration = makespan,
                           .isIntergreen = false});
    }
    
    for (auto gid : fromOnly) {
        auto& ivs = tr.transitions[gid].intervals;
        const auto& clearance = params.at(gid).to;
        const int cd = std::min(clearance.duration, makespan);
        const int ge = std::min(greenEnd.at(gid), makespan - cd);
        if (ge > 0)
            ivs.push_back({.aspect = from.groups.at(gid).aspect,
                           .duration = ge,
                           .isIntergreen = false});
        if (cd > 0)
            ivs.push_back({.aspect = clearance.aspect,
                           .duration = cd,
                           .isIntergreen = true});
        if (makespan - ge - cd > 0)
            ivs.push_back({.aspect = to.groups.at(gid).aspect,
                           .duration = makespan - ge - cd,
                           .isIntergreen = true});
    }
    for (auto gid : toOnly) {
        auto& ivs = tr.transitions[gid].intervals;
        const auto& startup = params.at(gid).from;
        const int sd = std::min(startup.duration, makespan);
        const int st = std::clamp(startTime.at(gid), sd, makespan);
        if (st - sd > 0)
            ivs.push_back({.aspect = from.groups.at(gid).aspect,
                           .duration = st - sd,
                           .isIntergreen = true});
        if (sd > 0)
            ivs.push_back({.aspect = startup.aspect,
                           .duration = sd,
                           .isIntergreen = true});
        if (makespan - st > 0)
            ivs.push_back({.aspect = to.groups.at(gid).aspect,
                           .duration = makespan - st,
                           .isIntergreen = false});
    }
    for (auto gid : neither) {
        auto& ivs = tr.transitions[gid].intervals;
        if (makespan > 0)
            ivs.push_back({.aspect = from.groups.at(gid).aspect,
                           .duration = makespan,
                           .isIntergreen = true});
    }
    return tr;
}

}  // namespace control
