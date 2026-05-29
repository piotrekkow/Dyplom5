#pragma once

#include "vector2.hpp"

struct Position {
    float x, y;
};

[[nodiscard]] constexpr Position operator+(Position p, Vector2 v) noexcept {
    return {p.x + v.dx, p.y + v.dy};
}
[[nodiscard]] constexpr Position operator-(Position p, Vector2 v) noexcept {
    return {p.x - v.dx, p.y - v.dy};
}
[[nodiscard]] constexpr Vector2 operator-(Position a, Position b) noexcept {
    return {a.x - b.x, a.y - b.y};
}

[[nodiscard]] bool operator==(Position a, Position b) noexcept;
[[nodiscard]] bool operator!=(Position a, Position b) noexcept;