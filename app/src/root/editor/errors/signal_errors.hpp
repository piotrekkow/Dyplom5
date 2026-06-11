#pragma once

#include <cstdint>

enum class SignalGroupError : uint8_t {
    // a cluster/crosswalk may belong to at most one group per config
    DuplicateClusterMembership,
    DuplicateCrosswalkMembership,
    // turn arrows: right-turn movements only, max one arrow group per movement
    MovementNotEligibleForTurnArrow,
    DuplicateTurnArrowForMovement
};
