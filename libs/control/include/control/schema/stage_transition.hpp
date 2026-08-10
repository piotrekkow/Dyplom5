#pragma once

#include <common/concepts.hpp>
#include <unordered_map>

#include "aspect_transition.hpp"

namespace control {

template <ComparableKey GroupKey>
struct StageTransition {
    std::unordered_map<GroupKey, AspectTransition> transitions;
    [[nodiscard]] int makespan() const {
        return transitions.empty() ? 0 : transitions.begin()->second.duration();
    }
};

}  // namespace control