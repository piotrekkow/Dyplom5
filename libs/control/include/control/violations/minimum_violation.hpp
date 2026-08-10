#pragma once

#include <common/concepts.hpp>

#include "control/schema/window.hpp"

namespace control {

template <ComparableKey GroupKey>
struct MinimumViolation {
    GroupKey group;
    Window culprit;
    int expected;
    int calculated;
};

}  // namespace control