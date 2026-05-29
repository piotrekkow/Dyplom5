#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <road_network/util/ids.hpp>

class MovementItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType + 3;
    [[nodiscard]] int type() const override { return Type; }

    explicit MovementItem(MovementId id, QPointF nodeCenter,
                          const std::vector<QPainterPath>& scenePaths);
    [[nodiscard]] MovementId id() const { return id_; }

    [[nodiscard]] QPainterPath shape() const override;
    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;

    // item local coordinate space for points
    void setPaths(const std::vector<QPainterPath>& scenePaths);

   private:
    MovementId id_;
    QRectF bounds_;
    std::vector<QPainterPath> paths_;

    inline static const QPen pathPen_{QColor("#aead64"), 0.1};
    void recomputeBounds();
};