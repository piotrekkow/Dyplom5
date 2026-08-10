#pragma once

#include <functional>

struct SolveHooks {
    std::function<void()> rederive, flush;
};