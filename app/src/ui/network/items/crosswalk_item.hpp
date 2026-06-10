#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <road_network/util/ids.hpp>

class CrosswalkItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType + 5;
    [[nodiscard]] int type() const override { return Type; }

    explicit CrosswalkItem(CrosswalkId id, QLineF l1, QLineF split, QLineF l2);
    [[nodiscard]] CrosswalkId id() const { return id_; }

    [[nodiscard]] QPainterPath shape() const override;
    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;
    void setLines(QLineF l1, QLineF split, QLineF l2);

   private:
    CrosswalkId id_;
    QRectF bounds_;
    QLineF l1_;
    QLineF split_;
    QLineF l2_;

    inline static const QPen line1Pen_{QColor("#004faf"), 0.3};
    inline static const QPen splitPen_{QColor("#2888ff"), 0.3, Qt::DotLine};
    inline static const QPen line2Pen_{QColor("#6f81c8"), 0.3};

    void recomputeBounds();
};