#pragma once

#include <QPainterPath>
#include <road_network/util/geometry/polyline.hpp>

struct CoordTransform {
    const bool yUp = true;  // set once, based on your domain convention

    [[nodiscard]] QPointF toScene(Position p) const {
        return {static_cast<qreal>(p.x),
                yUp ? -static_cast<qreal>(p.y) : static_cast<qreal>(p.y)};
    }

    [[nodiscard]] Position toDomain(QPointF p) const {
        return {float(p.x()), yUp ? float(-p.y()) : float(p.y())};
    }

    [[nodiscard]] QPointF toScene(Vector2 v) const {
        return {static_cast<qreal>(v.dx),
                yUp ? static_cast<qreal>(-v.dy) : static_cast<qreal>(v.dy)};
    }

    [[nodiscard]] QLineF toScene(Line l) {
        return {toScene(l.p1), toScene(l.p2)};
    }

    [[nodiscard]] QPainterPath toScene(const Polyline& p) const;
    [[nodiscard]] std::vector<QPointF> toScene(const std::vector<Position>& ps);
    [[nodiscard]] std::vector<QPainterPath> toScene(
        const std::vector<Polyline>& ps);
};