#pragma once

#include <cstdint>

enum class TimingError : uint8_t {
    IntervalsWontFitInCycle,
    CannotSetIntervalsOnAlwaysActiveSequence
};
