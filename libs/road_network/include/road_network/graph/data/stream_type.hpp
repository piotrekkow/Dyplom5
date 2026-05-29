#pragma once
#include <cstdint>

enum class CrosswalkType : uint8_t {
    PEDESTRIAN,
    CYCLIST,
    PEDESTRIAN_AND_CYCLIST
};
enum class StreamType : uint8_t {
    VEHICLE,
    PEDESTRIAN,
    CYCLIST /*, BUS, TRAM*/
};
enum class MovementType : uint8_t { VEHICLE /*, BUS, TRAM*/ };

inline StreamType toStreamType(MovementType t) {
    switch (t) {
        default:
        case MovementType::VEHICLE:
            return StreamType::VEHICLE;
    }
}

inline StreamType toStreamType(CrosswalkType t) {
    switch (t) {
        default:
        case CrosswalkType::PEDESTRIAN:
        case CrosswalkType::PEDESTRIAN_AND_CYCLIST:
            return StreamType::PEDESTRIAN;
        case CrosswalkType::CYCLIST:
            return StreamType::CYCLIST;
    }
}