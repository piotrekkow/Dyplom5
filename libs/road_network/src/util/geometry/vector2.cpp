#include <cassert>
#include <cmath>
#include <road_network/util/geometry/vector2.hpp>

#include "geom_eps.hpp"

Vector2 Vector2::fromAngle(float radians) noexcept {
    return {std::cos(radians), std::sin(radians)};
}

float Vector2::length() const noexcept { return std::hypot(dx, dy); }

float Vector2::angle() const noexcept {
    assert(!isZero() && "Cannot get angle of zero vector");
    return std::atan2(dy, dx);
}

Vector2 Vector2::normalized() const noexcept {
    const float len = length();
    assert(len > geomEps && "Cannot normalize zero vector");
    return {dx / len, dy / len};
}

Vector2 Vector2::rotate(float radians) const noexcept {
    const float c = std::cos(radians);
    const float s = std::sin(radians);
    return {dx * c - dy * s, dx * s + dy * c};
}

float Vector2::angleTo(Vector2 v) const noexcept {
    assert(!isZero() && !v.isZero() && "Cannot get angle to/from zero vector");
    return std::atan2(cross(v), dot(v));
}

bool Vector2::isZero() const noexcept {
    return dx * dx + dy * dy < geomEps * geomEps;
}

bool Vector2::isCollinear(Vector2 v) const noexcept {
    // |cross| / (|a| \cdot |b|) \approx sin \theta — pure angle test
    const float c = cross(v);
    const float denom = lengthSquared() * v.lengthSquared();  // |a|^2|b|^2
    return c * c <= geomEps * geomEps * denom;  // sin^2 \theta < \epsilon^2
}