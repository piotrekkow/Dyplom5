#pragma once

#include <cstdint>

enum class SignalAspect : uint8_t {
    RED,
    FLASHING_RED,
    RED_YELLOW,
    YELLOW,
    FLASHING_YELLOW,
    GREEN,
    FLASHING_GREEN,
    HORIZONTAL_DASH,
    HORIZONTAL_DOUBLE_DOT,
    VERTICAL_DASH,
    DARK
};