#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <road_network/util/ids.hpp>

class EntryItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType + 1;
    [[nodiscard]] int type() const override { return Type; }

    explicit EntryItem(EntryId id, QPointF origin, QPointF heading);
    [[nodiscard]] EntryId id() const { return id_; }

    [[nodiscard]] QPainterPath shape() const override;
    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;
    void setLanePoints(const std::vector<QPointF>& scenePoints);

   private:
    EntryId id_;
    QPointF heading_;
    QRectF bounds_;
    std::vector<QPointF> lanePoints_;

    inline static const QPen entryPen_{QColor("#78cc8c"), 0.3};
    static constexpr qreal handleRadius_ = 1.0;
    static constexpr qreal handleArrowLength_ = 3.0;
    static constexpr qreal handleArrowTipLength_ = 0.5;
    static constexpr qreal lanePointSize_ = 0.4;

    void recomputeBounds();
};