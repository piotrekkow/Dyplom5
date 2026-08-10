#pragma once

#include <common/matrix.hpp>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "control/timing_math.hpp"
#include "schema/group_params.hpp"
#include "schema/timing_plan.hpp"
#include "violations/intergreen_violation.hpp"
#include "violations/minimum_violation.hpp"

namespace control {

namespace detail {
[[nodiscard]] inline int signedClearance(int evacEnd, int evacLen,
                                         int apprStart, int apprLen, int C) {
    const int f = mod(apprStart - evacEnd, C);
    const int overlap =
        std::max(0, std::min(f + apprLen, C)
                        - std::max(f, C - evacLen));
    return overlap > 0 ? -overlap : f;
}
}  // namespace detail

template <ComparableKey GroupKey>
std::vector<IntergreenViolation<GroupKey>> validatePlanIntergreen(
    const TimingPlan<GroupKey>& plan, const Matrix<GroupKey, int>& intergreen,
    const std::unordered_map<GroupKey, GroupParams>& params) {
    std::vector<IntergreenViolation<GroupKey>> violations;
    const int C = plan.cycleLength();
    if (C <= 0) return violations;

    const auto windows = plan.activeWindows();
    auto windowsOf = [&](const GroupKey& g) -> const std::vector<Window>* {
        auto it = windows.find(g);
        return it == windows.end() ? nullptr : &it->second;
    };

    intergreen.forEach([&](const GroupKey& evac, const GroupKey& appr,
                           int required) {
        const auto* eivs = windowsOf(evac);
        const auto* aivs = windowsOf(appr);
        if (!eivs || !aivs) return;

        const auto& toSwitch = params.at(evac).to;      // clearance after evac
        const auto& fromSwitch = params.at(appr).from;  // startup before appr
        const int evacAfter = toSwitch.isIntergreen ? 0 : toSwitch.duration;
        const int apprDelay = fromSwitch.isIntergreen ? 0 : fromSwitch.duration;

        for (const Window& eiv : *eivs) {
            auto [es, eLen] = intervalSpan(eiv, C);
            const int evacEnd = mod(es + eLen + evacAfter, C);
            const int evacLen = eLen + evacAfter;
            for (const Window& aiv : *aivs) {
                auto [as, aLen] = intervalSpan(aiv, C);
                const int apprStart = mod(as + apprDelay, C);
                const int apprLen = std::max(0, aLen - apprDelay);
                const int clearance = detail::signedClearance(
                    evacEnd, evacLen, apprStart, apprLen, C);
                if (clearance < required)
                    violations.push_back(
                        {evac, appr, eiv, aiv, required, clearance});
            }
        }
    });
    return violations;
}

template <ComparableKey GroupKey>
std::vector<MinimumViolation<GroupKey>> validatePlanMinimums(
    const TimingPlan<GroupKey>& plan,
    const std::unordered_map<GroupKey, GroupParams>& params) {
    std::vector<MinimumViolation<GroupKey>> violations;
    const int C = plan.cycleLength();
    if (C <= 0) return violations;

    for (const auto& [group, ivs] : plan.activeWindows()) {
        int required = params.at(group).minActiveTime;
        for (const Window& iv : ivs) {
            if (int l = intervalLength(iv, C); l < required)
                violations.push_back({group, iv, required, l});
        }
    }
    return violations;
}

// returns empty if valid, returns invalid pairings if invalid. active denotes
// groups active in stage
template <ComparableKey GroupKey>
std::vector<std::pair<GroupKey, GroupKey>> stageIntergreenBreaches(
    const Matrix<GroupKey, int>& intergreen,
    const std::unordered_set<GroupKey>& active) {
    std::vector<std::pair<GroupKey, GroupKey>> bad;
    for (auto g : active) {
        for (auto other : active) {
            if (g == other) continue;
            if (intergreen.find(g, other)) bad.emplace_back(g, other);
        }
    }
    return bad;
}

}  // namespace control
