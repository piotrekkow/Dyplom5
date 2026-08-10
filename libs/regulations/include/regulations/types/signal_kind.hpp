#pragma once

#include <cstdint>

#include "stream_kind.hpp"

enum class SignalKind : uint8_t {
    VEHICLE,
    CONDITIONAL_ARROW,
    PEDESTRIAN,
    CYCLIST,
    PEDESTRIAN_AND_CYCLIST,
    BUS,
    TRAM,
    count
};

inline SignalKind toSignalKind(CrosswalkType t) {
    switch (t) {
        default:
        case CrosswalkType::PEDESTRIAN:
            return SignalKind::PEDESTRIAN;
        case CrosswalkType::CYCLIST:
            return SignalKind::CYCLIST;
        case CrosswalkType::PEDESTRIAN_AND_CYCLIST:
            return SignalKind::PEDESTRIAN_AND_CYCLIST;
    }
}

inline SignalKind toSignalKind(MovementType t) {
    switch (t) {
        default:
        case MovementType::VEHICLE:
            return SignalKind::VEHICLE;
        case MovementType::BUS:
            return SignalKind::BUS;
        case MovementType::TRAM:
            return SignalKind::TRAM;
    }
}