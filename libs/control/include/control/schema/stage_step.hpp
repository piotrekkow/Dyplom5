#pragma once

#include <common/concepts.hpp>

#include "stage.hpp"
#include "stage_transition.hpp"

namespace control {

template <ComparableKey GroupKey>
struct StageStep {
    Stage<GroupKey> stage;
    StageTransition<GroupKey> transition;
    int stageDuration;
    [[nodiscard]] int duration() const {
        return stageDuration + transition.makespan();
    }
};

}  // namespace control