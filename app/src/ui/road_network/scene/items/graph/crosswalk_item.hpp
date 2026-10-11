#pragma once

#include <QGraphicsItem>
#include <road_network/util/ids.hpp>

class CrosswalkItem : public QGraphicsItem {
   public:
    explicit CrosswalkItem(CrosswalkId id);

    [[nodiscard]] CrosswalkId id() const { return id_; }

    void setGeometry(QLineF anchor, QLineF split, QLineF edge);

    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    [[nodiscard]] QPainterPath shape() const override { return hitShape_; }
    void paint(QPainter* p, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

   private:
    CrosswalkId id_;
    QLineF anchor_;
    QLineF edge_;
    QLineF splitOffset_;
    bool drawEdge_ = false;
    bool drawSplitOffset_ = false;
    QPolygonF body_;
    QPainterPath hitShape_;
    QRectF bounds_;

    void paintStripes(QPainter* p, const QColor& color) const;
};
