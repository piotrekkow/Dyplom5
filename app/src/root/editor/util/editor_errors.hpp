#pragma once

#include <cstdint>

enum class MovementLayoutError : uint8_t {
    EmptyLaneSet,       // No lanes in entry
    SpecSizeMismatch,   // Specs and shared vectors size don't allow for layout
                        // instantiation. Shared requires one less element than
                        // specs to decide whether each next spec shares lane
                        // with last
    LaneCountMismatch,  // More lane specs provided than lanes in movement
    AngularOrderViolation,  // Supplied exits in spec vector were not sorted
                            // from leftmost to rightmost relative to entry
                            // heading
    ExitLaneOverflow  // A movement is given more entry lanes than its target
                      // exit has lanes
};

enum class SignalGroupErrors : uint8_t {
    CandidateMovementsOfMultipleTypes,  // Groups have to be made up of a single
                                        // type
    CandidateCrosswalksOfMultipleSeries,  // Single series ensures variant above
                                          // and provides a consistant minimum
    ProtectionEditOnNonMovementGroup
};

enum class StageErrors : uint8_t {
    ContainsPairsDisallowedByIntergreenMatrix,
    GroupsFromOutsideNodeSupplied
};

enum class TimingErrors : uint8_t {
    StepWithInvalidStageAndTransitionPair,
    NextTransitionDoesntMatchStep
};