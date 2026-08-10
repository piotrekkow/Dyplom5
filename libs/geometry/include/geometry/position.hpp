#pragma once

#include "vector2.hpp"

struct Position {
    double x, y;
};

[[nodiscard]] constexpr Position operator+(Position p, Vector2 v) noexcept {
    return {.x = p.x + v.dx, .y = p.y + v.dy};
}
[[nodiscard]] constexpr Position operator-(Position p, Vector2 v) noexcept {
    return {.x = p.x - v.dx, .y = p.y - v.dy};
}
[[nodiscard]] constexpr Vector2 operator-(Position a, Position b) noexcept {
    return {.dx = a.x - b.x, .dy = a.y - b.y};
}

[[nodiscard]] bool operator==(Position a, Position b) noexcept;
[[nodiscard]] bool operator!=(Position a, Position b) noexcept;