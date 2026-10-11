#pragma once

#include <QGraphicsItem>
#include <road_network/util/ids.hpp>

class CrosswalkSeriesItem : public QGraphicsItem {
   public:
    explicit CrosswalkSeriesItem(CrosswalkSeriesId id);

    [[nodiscard]] CrosswalkSeriesId id() const { return id_; }

    void setPath(const QPainterPath& scenePath);

    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    [[nodiscard]] QPainterPath shape() const override { return hitShape_; }
    void paint(QPainter* p, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

   private:
    CrosswalkSeriesId id_;
    QPainterPath path_;      // pedestrian path, painted
    QPainterPath hitShape_;  // stroked outline, for picking
    QRectF bounds_;
};
