#pragma once

#include "position.hpp"

struct Line {
    Position p1;
    Position p2;

    // @return true if lines intersect, false otherwise
    // @param intersection output parameter for intersection point
    bool intersection(const Line& other, Position& intersection) const;
    [[nodiscard]] float length() const;
    [[nodiscard]] Vector2 direction() const;
};