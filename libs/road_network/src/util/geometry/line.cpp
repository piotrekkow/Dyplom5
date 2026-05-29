#include <cmath>
#include <road_network/util/geometry/line.hpp>

#include "geom_eps.hpp"

namespace {
struct LineEquation {
    float a, b, c;  // ax + by + c = 0
};

LineEquation toEquation(const Line& line) {
    float x1 = line.p1.x;
    float y1 = line.p1.y;
    float x2 = line.p2.x;
    float y2 = line.p2.y;

    float a = y1 - y2;
    float b = x2 - x1;
    float c = x1 * y2 - x2 * y1;

    return {a, b, c};
}

}  // namespace

bool Line::intersection(const Line& other, Position& intersection) const {
    auto eq1 = toEquation(*this);
    auto eq2 = toEquation(other);

    float det = eq1.a * eq2.b - eq2.a * eq1.b;
    if (std::abs(det) < geomEps) return false;

    float x = (eq1.b * eq2.c - eq2.b * eq1.c) / det;
    float y = (eq1.c * eq2.a - eq2.c * eq1.a) / det;

    intersection = Position(x, y);
    return true;
}

float Line::length() const { return (p2 - p1).length(); }
Vector2 Line::direction() const { return p2 - p1; }