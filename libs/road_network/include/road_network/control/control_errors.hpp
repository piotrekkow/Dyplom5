#pragma once

#include <cstdint>

enum class TimingError : uint8_t {
    IntervalMinViolation,
    IntervalsWontFitInCycle,
    CannotSetIntervalsOnAlwaysActiveSequence
};

enum class TimingWarning : uint8_t { UpdatedMinIntervalViolation };