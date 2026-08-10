#pragma once

#include <QGraphicsItem>
#include <road_network/util/ids.hpp>

class NodeItem : public QGraphicsItem {
   public:
    explicit NodeItem(NodeId id);

    [[nodiscard]] NodeId id() const { return id_; }

    void setCenter(QPointF sceneCenter) { setPos(sceneCenter); }

    [[nodiscard]] QRectF boundingRect() const override;
    [[nodiscard]] QPainterPath shape() const override;
    void paint(QPainter* p, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

   private:
    NodeId id_;

    static constexpr qreal kRadius = 10.0;  // meters
};
