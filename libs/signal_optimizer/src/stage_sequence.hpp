#pragma once
#include <control/schema/stage.hpp>
#include <control/schema/stage_transition.hpp>
#include <regulations/regulations.hpp>
#include <vector>

#include "types.hpp"

// A candidate cyclic ordering of a stage cover: the stages in order, and the
// transition following each one (transitions[i] is between stages[i] and
// stages[(i+1) % size])
struct SequenceCandidate {
    std::vector<control::Stage<GId>> stages;
    std::vector<control::StageTransition<GId>> transitions;
};

// Returns all (k-1)! circular orderings of the given cover, with transitions
// computed via control::computeTransitionWeighted.
std::vector<SequenceCandidate> enumerateSequences(
    const std::vector<GStage>& cover, const GConfig& cfg,
    const Regulations& regulations);
