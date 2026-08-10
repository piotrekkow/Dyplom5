#pragma once

#include <variant>

struct LineParams {};
struct QuadBezierParams {
};  // control point is derived from entry and exit line intersection
struct CubicBezierParams {
    double cE;  // first control point offset along entry line
    double cX;  // second control point offset along exit line
};

using MovementGeometryParams =
    std::variant<LineParams, QuadBezierParams, CubicBezierParams>;

struct MovementPathSpec {
    double dE;  // offset along entry line
    double dX;  // offset along exit line
    MovementGeometryParams params;
};