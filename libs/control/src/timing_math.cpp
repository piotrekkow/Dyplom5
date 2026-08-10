#include "control/timing_math.hpp"

#include "control/schema/aspect_interval.hpp"
#include "control/schema/window.hpp"

namespace control {

std::vector<Window> activeWindows(const std::vector<AspectInterval>& sequence) {
    std::vector<Window> windows;
    int t = 0;
    for (const auto& interval : sequence) {
        if (!interval.isIntergreen) {
            if (!windows.empty() && windows.back().end == t)
                windows.back().end += interval.duration;
            else
                windows.push_back({.onset = t, .end = t + interval.duration});
        }
        t += interval.duration;
    }
    if (windows.size() > 1 && windows.front().onset == 0 &&
        windows.back().end == t) {
        windows.front().onset = windows.back().onset;
        windows.pop_back();
    }
    return windows;
}

}  // namespace control
