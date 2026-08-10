#pragma once

#include <QGraphicsItem>
#include <road_network/util/ids.hpp>

class MovementItem : public QGraphicsItem {
   public:
    explicit MovementItem(MovementId id);

    [[nodiscard]] MovementId id() const { return id_; }

    void setPaths(const std::vector<QPainterPath>& scenePaths);

    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    [[nodiscard]] QPainterPath shape() const override;
    void paint(QPainter* p, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

   private:
    MovementId id_;
    QPainterPath path_;      // united lane paths, painted outside Demand
    QPainterPath hitShape_;  // stroked outline (all lanes)
    QRectF bounds_;

    void rebuildGeometry();
};
