#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <road_network/util/ids.hpp>

class NodeItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType;
    [[nodiscard]] int type() const override { return Type; }

    explicit NodeItem(NodeId id, QPointF center);
    [[nodiscard]] NodeId id() const { return id_; }

    [[nodiscard]] QPainterPath shape() const override;
    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;

    // item local coordinate space for points
    void setConflictPoints(const std::vector<QPointF>& scenePoints);

   private:
    NodeId id_;
    QRectF bounds_;
    std::vector<QPointF> conflictPoints_;

    inline static const QPen handlePen_{QColor("#1fcccc78"), 2.0};
    inline static const QBrush handleBrush_{QColor("#1f8c8752")};
    inline static const QPen conflictPen_{QColor("#FF0000"), 0.05};
    static constexpr qreal handleRadius_ = 10.0;
    static constexpr qreal conflictPointSize_ = 0.4;

    void recomputeBounds();
};