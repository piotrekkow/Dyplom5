#pragma once

#include <utility>
#include <vector>

#include "control/schema/window.hpp"

namespace control {

struct AspectInterval;

// Positive modulo, for reading cyclic offsets.
constexpr int mod(int x, int m) { return ((x % m) + m) % m; }

// Length of an interval in cycle-seconds. An Interval is half-open
// [onset, end); a wrapping interval (spanning the cycle boundary) is encoded as
// onset > end, and both cases read uniformly through here.
constexpr int intervalLength(const Window& iv, int cycle) {
    return iv.onset <= iv.end ? iv.end - iv.onset : cycle - iv.onset + iv.end;
}

// Window as (onset normalized to [0, cycle), length).
inline std::pair<int, int> intervalSpan(const Window& iv, int cycle) {
    return {mod(iv.onset, cycle), intervalLength(iv, cycle)};
}

std::vector<Window> activeWindows(const std::vector<AspectInterval>& sequence);

}  // namespace control
