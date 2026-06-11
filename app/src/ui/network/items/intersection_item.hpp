#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <road_network/util/ids.hpp>

// Control-plane boundary of an intersection, bounded on the geometry of its
// member nodes. Clicking anywhere inside selects the intersection.
class IntersectionItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType + 7;
    [[nodiscard]] int type() const override { return Type; }

    IntersectionItem(IntersectionId id, const QPainterPath& sceneOutline);
    [[nodiscard]] IntersectionId id() const { return id_; }

    [[nodiscard]] QPainterPath shape() const override { return outline_; }
    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;

    void setOutline(const QPainterPath& sceneOutline);

   private:
    IntersectionId id_;
    QPainterPath outline_;
    QRectF bounds_;

    inline static const QPen outlinePen_{QColor("#9c7bd6"), 0.5, Qt::DashLine};
    inline static const QBrush fillBrush_{QColor(124, 96, 176, 45)};
};
