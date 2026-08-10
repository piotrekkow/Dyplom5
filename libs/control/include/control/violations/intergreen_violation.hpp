#pragma once

#include <common/concepts.hpp>

#include "control/schema/window.hpp"

namespace control {

template <ComparableKey GroupKey>
struct IntergreenViolation {
    GroupKey evac;
    GroupKey appr;
    Window culpritEvac;
    Window culpritAppr;
    int expected;
    int calculated;
};

}  // namespace control