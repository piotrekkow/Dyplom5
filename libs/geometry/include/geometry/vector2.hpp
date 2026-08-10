#pragma once

struct Vector2 {
    double dx, dy;

    [[nodiscard]] static Vector2 fromAngle(double radians) noexcept;
    [[nodiscard]] bool isZero() const noexcept;
    [[nodiscard]] constexpr double lengthSquared() const noexcept {
        return dx * dx + dy * dy;
    }
    [[nodiscard]] double length() const noexcept;
    [[nodiscard]] double angle() const noexcept;
    [[nodiscard]] Vector2 normalized() const noexcept;
    [[nodiscard]] Vector2 rotate(double radians) const noexcept;
    [[nodiscard]] constexpr double dot(Vector2 v) const noexcept {
        return dx * v.dx + dy * v.dy;
    }
    [[nodiscard]] constexpr double cross(Vector2 v) const noexcept {
        return dx * v.dy - dy * v.dx;
    }
    // Returns signed angle from this → v (positive = CCW in y-up / math
    // coords). In y-down screen coords right turns are positive — negate if
    // needed.
    [[nodiscard]] double angleTo(Vector2 v) const noexcept;
    // perpendicular counter clockwise
    [[nodiscard]] Vector2 perpCcw() const noexcept {
        return {.dx = -dy, .dy = dx};
    }
    // perpendicular clockwise
    [[nodiscard]] Vector2 perpCw() const noexcept {
        return {.dx = dy, .dy = -dx};
    }
    [[nodiscard]] bool isCollinear(Vector2 v) const noexcept;
};

[[nodiscard]] constexpr Vector2 operator+(Vector2 a, Vector2 b) noexcept {
    return {.dx = a.dx + b.dx, .dy = a.dy + b.dy};
}
[[nodiscard]] constexpr Vector2 operator-(Vector2 a, Vector2 b) noexcept {
    return {.dx = a.dx - b.dx, .dy = a.dy - b.dy};
}
[[nodiscard]] constexpr Vector2 operator*(Vector2 v, double s) noexcept {
    return {.dx = v.dx * s, .dy = v.dy * s};
}
[[nodiscard]] constexpr Vector2 operator*(double s, Vector2 v) noexcept {
    return v * s;
}
[[nodiscard]] constexpr Vector2 operator/(Vector2 v, double s) noexcept {
    return {.dx = v.dx / s, .dy = v.dy / s};
}
[[nodiscard]] constexpr Vector2 operator-(Vector2 v) noexcept {
    return {.dx = -v.dx, .dy = -v.dy};
}

[[nodiscard]] constexpr bool operator==(Vector2 a, Vector2 b) noexcept {
    return a.dx == b.dx && a.dy == b.dy;
}
[[nodiscard]] constexpr bool operator!=(Vector2 a, Vector2 b) noexcept {
    return !(a == b);
}
