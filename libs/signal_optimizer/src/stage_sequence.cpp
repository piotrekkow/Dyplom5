#include "stage_sequence.hpp"

#include <algorithm>
#include <control/schema/group_params.hpp>
#include <control/transition.hpp>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

#include "types.hpp"

namespace {
SignalKind kindOf(GKind kind) {
    return kind == GKind::Crosswalk ? SignalKind::PEDESTRIAN
                                    : SignalKind::VEHICLE;
}

// Builds control::GroupParams for every group in cfg from its aspect timing
std::unordered_map<GId, control::GroupParams> buildGroupParams(
    const GConfig& cfg, const Regulations& regulations) {
    std::unordered_map<GId, control::GroupParams> params;
    for (const auto& g : cfg.groups) {
        const auto& a = regulations.aspects(kindOf(g.kind));
        // .from is read for groups newly active in the "to" stage; .to is read
        // for groups evacuating the "from" stage
        params[g.id] = control::GroupParams{
            .minActiveTime = std::max(g.tMin, a.minGreen.fixed),
            .from = {.aspect = a.toActive.aspect,
                     .duration = a.toActive.duration,
                     .isIntergreen = a.toActive.isIntergreen},
            .to = {.aspect = a.toInactive.aspect,
                   .duration = a.toInactive.duration,
                   .isIntergreen = a.toInactive.isIntergreen},
        };
    }
    return params;
}

// Builds a dense control::Stage over every group in cfg (not just the active
// ones): control::computeTransitionWeighted needs every group present to
// classify from-only / to-only / both-active / neither across a transition.
control::Stage<GId> buildStage(const GStage& stage, const GConfig& cfg,
                               const Regulations& regulations) {
    std::unordered_set<GId> active(stage.groupIds.begin(),
                                   stage.groupIds.end());
    control::Stage<GId> cStage;
    for (const auto& g : cfg.groups) {
        const auto& a = regulations.aspects(kindOf(g.kind));
        bool isActive = active.contains(g.id);
        cStage.groups[g.id] = {.aspect = isActive ? a.active : a.inactive,
                               .isActive = isActive};
    }
    return cStage;
}
}  // namespace

std::vector<SequenceCandidate> enumerateSequences(
    const std::vector<GStage>& cover, const GConfig& cfg,
    const Regulations& regulations) {
    int k = static_cast<int>(cover.size());
    if (k == 0) return {};

    auto groupParams = buildGroupParams(cfg, regulations);
    std::unordered_map<GId, double> weights;
    for (const auto& g : cfg.groups) weights[g.id] = g.maxY;

    std::vector<control::Stage<GId>> stages;
    stages.reserve(cover.size());
    for (const auto& stage : cover)
        stages.push_back(buildStage(stage, cfg, regulations));

    std::vector<SequenceCandidate> result;

    // Fix stage at index 0, permute the rest
    std::vector<int> perm(static_cast<std::size_t>(k - 1));
    std::ranges::iota(perm, 1);

    do {
        std::vector<int> order = {0};
        order.insert(order.end(), perm.begin(), perm.end());

        SequenceCandidate candidate;
        candidate.stages.reserve(static_cast<std::size_t>(k));
        for (int idx : order)
            candidate.stages.push_back(stages[static_cast<std::size_t>(idx)]);

        candidate.transitions.resize(static_cast<std::size_t>(k));
        for (int i = 0; i < k; i++) {
            const auto& from = candidate.stages[static_cast<std::size_t>(i)];
            const auto& to =
                candidate.stages[static_cast<std::size_t>((i + 1) % k)];
            candidate.transitions[static_cast<std::size_t>(i)] =
                control::computeTransitionWeighted<GId>(
                    from, to, groupParams, cfg.intergreen, weights);
        }
        result.push_back(std::move(candidate));
    } while (std::next_permutation(perm.begin(), perm.end()));

    return result;
}
