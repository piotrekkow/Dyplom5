#include "geometry/vector2.hpp"

#include <cassert>
#include <cmath>

#include "geometry_epsilon.hpp"

Vector2 Vector2::fromAngle(double radians) noexcept {
    return {.dx = std::cos(radians), .dy = std::sin(radians)};
}

double Vector2::length() const noexcept { return std::hypot(dx, dy); }

double Vector2::angle() const noexcept {
    assert(!isZero() && "Cannot get angle of zero vector");
    return std::atan2(dy, dx);
}

Vector2 Vector2::normalized() const noexcept {
    const double len = length();
    assert(len > geometryEpsilon && "Cannot normalize zero vector");
    return {.dx = dx / len, .dy = dy / len};
}

Vector2 Vector2::rotate(double radians) const noexcept {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return {.dx = dx * c - dy * s, .dy = dx * s + dy * c};
}

double Vector2::angleTo(Vector2 v) const noexcept {
    assert(!isZero() && !v.isZero() && "Cannot get angle to/from zero vector");
    return std::atan2(cross(v), dot(v));
}

bool Vector2::isZero() const noexcept {
    return dx * dx + dy * dy < geometryEpsilon * geometryEpsilon;
}

bool Vector2::isCollinear(Vector2 v) const noexcept {
    // |cross| / (|a| \cdot |b|) \approx sin \theta, pure angle test
    const double c = cross(v);
    const double denom = lengthSquared() * v.lengthSquared();  // |a|^2|b|^2
    return c * c <= geometryEpsilon * geometryEpsilon *
                        denom;  // sin^2 \theta < \epsilon^2
}