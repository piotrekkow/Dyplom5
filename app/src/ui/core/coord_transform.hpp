#pragma once

#include <QLineF>
#include <QPainterPath>
#include <QPointF>
#include <geometry/line.hpp>
#include <geometry/polyline.hpp>
#include <geometry/position.hpp>
#include <geometry/vector2.hpp>
#include <vector>

// Domain-scene mapping. The domain is y-up, Qt scenes are y-down.
struct CoordTransform {
    const bool yUp = true;

    [[nodiscard]] QPointF toScene(Position p) const {
        return {p.x, yUp ? -p.y : p.y};
    }

    [[nodiscard]] Position toDomain(QPointF p) const {
        return {.x = p.x(), .y = yUp ? -p.y() : p.y()};
    }

    [[nodiscard]] QPointF toScene(Vector2 v) const {
        return {v.dx, yUp ? -v.dy : v.dy};
    }

    [[nodiscard]] QLineF toScene(Line l) const {
        return {toScene(l.p1), toScene(l.p2)};
    }

    [[nodiscard]] QPainterPath toScene(const Polyline& p) const;
    [[nodiscard]] std::vector<QPointF> toScene(
        const std::vector<Position>& ps) const;
    [[nodiscard]] std::vector<QPainterPath> toScene(
        const std::vector<Polyline>& ps) const;
};
