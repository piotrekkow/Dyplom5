#include "coord_transform.hpp"

#include <road_network/util/overloaded.hpp>
#include <variant>

QPainterPath CoordTransform::toScene(const Polyline& p) const {
    QPainterPath path;
    path.moveTo(toScene(p.start()));
    for (const auto& seg : p.segments()) {
        std::visit(
            overloaded{
                [&](const Polyline::LineSeg& s) { path.lineTo(toScene(s.to)); },
                [&](const Polyline::QuadSeg& s) {
                    path.quadTo(toScene(s.ctrl), toScene(s.to));
                },
                [&](const Polyline::CubicSeg& s) {
                    path.cubicTo(toScene(s.c1), toScene(s.c2), toScene(s.to));
                },
            },
            seg);
    }
    return path;
}

std::vector<QPointF> CoordTransform::toScene(const std::vector<Position>& ps) {
    std::vector<QPointF> out;
    out.reserve(ps.size());
    for (const auto& p : ps) out.push_back(toScene(p));
    return out;
}

std::vector<QPainterPath> CoordTransform::toScene(
    const std::vector<Polyline>& ps) {
    std::vector<QPainterPath> out;
    out.reserve(ps.size());
    for (const auto& p : ps) out.push_back(toScene(p));
    return out;
}