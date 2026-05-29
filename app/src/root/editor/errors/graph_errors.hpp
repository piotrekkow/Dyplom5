#pragma once

#include <cstdint>

enum class MovementLayoutError : uint8_t {
    SpecSizeMismatch,
    LaneCountMismatch,
    AngularOrderViolation,
    EmptyLaneSet
};

enum class MovementSplitError : uint8_t {
    LaneCountToSplitMismatch,
    ExitLaneCountSplitMismatch
};