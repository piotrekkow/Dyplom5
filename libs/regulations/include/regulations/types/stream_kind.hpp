#pragma once

#include <cstdint>

enum class MovementType : uint8_t { VEHICLE, BUS, TRAM };

enum class CrosswalkType : uint8_t {
    PEDESTRIAN,
    CYCLIST,
    PEDESTRIAN_AND_CYCLIST
};

enum class StreamKind : uint8_t {
    VEHICLE,
    CYCLIST,
    PEDESTRIAN,
    BUS,
    TRAM,
    count
};

inline StreamKind toStreamKind(MovementType t) {
    switch (t) {
        default:
        case MovementType::VEHICLE:
            return StreamKind::VEHICLE;
    }
}

inline StreamKind toStreamKind(CrosswalkType t) {
    switch (t) {
        default:
        case CrosswalkType::PEDESTRIAN:
        case CrosswalkType::PEDESTRIAN_AND_CYCLIST:
            return StreamKind::PEDESTRIAN;
        case CrosswalkType::CYCLIST:
            return StreamKind::CYCLIST;
    }
}