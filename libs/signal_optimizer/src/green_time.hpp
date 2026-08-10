#pragma once
#include <control/schema/stage.hpp>
#include <control/schema/stage_transition.hpp>
#include <optional>

#include "types.hpp"

// Distributes slack green time one second at a time, always extending the
// stage that yields the greatest marginal reduction in total delay.
// Returns nullopt if transition overhead alone exceeds T. `stages` and
// `transitions` are aligned (transitions[i] leads into stages[i]).
std::optional<GGreen> greedyMarginalAlloc(
    const std::vector<control::Stage<GId>>& stages,
    const std::vector<control::StageTransition<GId>>& transitions,
    const GConfig& cfg, int T);
