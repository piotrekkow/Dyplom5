#pragma once

#include <QGraphicsItem>

class BoundaryItem : public QGraphicsItem {
   public:
    // Scene position of the bar's origin, rotation mapping +X onto the
    // heading, and the bar's length (total lane width).
    void setGeometry(QPointF scenePos, qreal angleDeg, qreal width) {
        prepareGeometryChange();
        setPos(scenePos);
        setRotation(angleDeg);
        width_ = width;
        update();
    }

   protected:
    BoundaryItem();

    // The bar across the lanes, from the origin along +Y.
    void paintBar(QPainter* p, const QColor& color) const;
    // An arrow along +X: tail at tailX, head tip at tailX + kArrowLength.
    static void paintArrow(QPainter* p, const QColor& color, qreal tailX);

    qreal width_ = 3.5;

    static constexpr qreal kArrowLength = 6.0;  // meters
    static constexpr qreal kHitMargin = 1.0;    // meters
};
