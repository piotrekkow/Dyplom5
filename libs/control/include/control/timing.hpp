#pragma once

#include <cassert>
#include <vector>

#include "schema/stage_sequence.hpp"
#include "schema/timing_plan.hpp"

namespace control {

namespace detail {
// Doesn't wrap the first/last interval across the cycle boundary.
inline void mergeAdjacentAspectIntervals(std::vector<AspectInterval>& seq) {
    std::vector<AspectInterval> merged;
    merged.reserve(seq.size());
    for (const AspectInterval& iv : seq) {
        if (!merged.empty() && merged.back().aspect == iv.aspect &&
            merged.back().isIntergreen == iv.isIntergreen)
            merged.back().duration += iv.duration;
        else
            merged.push_back(iv);
    }
    seq = std::move(merged);
}
}  // namespace detail

template <ComparableKey GroupKey>
TimingPlan<GroupKey> buildTimingPlan(const StageSequence<GroupKey>& seq) {
    TimingPlan<GroupKey> plan;
    if (seq.empty()) return plan;

    for (const auto& [g, _] : seq.front().stage.groups) {
        std::vector<AspectInterval>& out = plan.sequences[g];
        for (const auto& step : seq) {
            const auto& curr = step.stage;
            auto cIt = curr.groups.find(g);
            assert(cIt != curr.groups.end());
            const StageAspect& sa = cIt->second;
            const int makespan = step.transition.makespan();

            // stage
            out.push_back({.aspect = sa.aspect,
                           .duration = step.stageDuration,
                           .isIntergreen = !sa.isActive});

            // transition
            if (makespan > 0) {
                const auto& transitions = step.transition.transitions;
                auto tIt = transitions.find(g);
                if (tIt == transitions.end()) assert(false);

                const auto& ivs = tIt->second.intervals;
                out.insert(out.end(), ivs.begin(), ivs.end());
            }
        }
        detail::mergeAdjacentAspectIntervals(out);
    }
    return plan;
}

}  // namespace control
