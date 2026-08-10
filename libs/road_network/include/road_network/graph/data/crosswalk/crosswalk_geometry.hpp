#pragma once

#include <geometry/line.hpp>

struct CrosswalkGeometryInput {
    struct Segment {
        double width, x1, x2;
    };
    Line anchor;
    Segment split;
    Segment edge;
};

struct CrosswalkGeometry {
    Line anchor;
    Line split;
    Line edge;
};