#pragma once

struct Vector2 {
    float dx, dy;
    [[nodiscard]] static Vector2 fromAngle(float radians) noexcept;
    [[nodiscard]] bool isZero() const noexcept;
    [[nodiscard]] constexpr float lengthSquared() const noexcept {
        return dx * dx + dy * dy;
    }
    [[nodiscard]] float length() const noexcept;
    [[nodiscard]] float angle() const noexcept;
    [[nodiscard]] Vector2 normalized() const noexcept;
    [[nodiscard]] Vector2 rotate(float radians) const noexcept;
    [[nodiscard]] constexpr float dot(Vector2 v) const noexcept {
        return dx * v.dx + dy * v.dy;
    }
    [[nodiscard]] constexpr float cross(Vector2 v) const noexcept {
        return dx * v.dy - dy * v.dx;
    }
    // Returns signed angle from this → v (positive = CCW in y-up / math
    // coords). In y-down screen coords right turns are positive — negate if
    // needed.
    [[nodiscard]] float angleTo(Vector2 v) const noexcept;
    // perpendicular counter clockwise
    [[nodiscard]] Vector2 perpCcw() const noexcept { return {-dy, dx}; }
    // perpendicular clockwise
    [[nodiscard]] Vector2 perpCw() const noexcept { return {dy, -dx}; }
    [[nodiscard]] bool isCollinear(Vector2 v) const noexcept;
};

[[nodiscard]] constexpr Vector2 operator+(Vector2 a, Vector2 b) noexcept {
    return {a.dx + b.dx, a.dy + b.dy};
}
[[nodiscard]] constexpr Vector2 operator-(Vector2 a, Vector2 b) noexcept {
    return {a.dx - b.dx, a.dy - b.dy};
}
[[nodiscard]] constexpr Vector2 operator*(Vector2 v, float s) noexcept {
    return {v.dx * s, v.dy * s};
}
[[nodiscard]] constexpr Vector2 operator*(float s, Vector2 v) noexcept {
    return v * s;
}
[[nodiscard]] constexpr Vector2 operator/(Vector2 v, float s) noexcept {
    return {v.dx / s, v.dy / s};
}
[[nodiscard]] constexpr Vector2 operator-(Vector2 v) noexcept {
    return {-v.dx, -v.dy};
}

[[nodiscard]] constexpr bool operator==(Vector2 a, Vector2 b) noexcept {
    return a.dx == b.dx && a.dy == b.dy;
}
[[nodiscard]] constexpr bool operator!=(Vector2 a, Vector2 b) noexcept {
    return !(a == b);
}
