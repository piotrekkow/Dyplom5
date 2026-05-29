#include "crosswalk_series_item.hpp"

#include <QPainter>

CrosswalkSeriesItem::CrosswalkSeriesItem(CrosswalkSeriesId id) : id_(id) {}

QPainterPath CrosswalkSeriesItem::shape() const { return pedestrianPath_; }

void CrosswalkSeriesItem::paint(QPainter* painter,
                                const QStyleOptionGraphicsItem* option,
                                QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    if (pedestrianPath_.isEmpty()) return;

    QPen pen = pathPen_;
    painter->setPen(pen);
    painter->drawPath(pedestrianPath_);
}

void CrosswalkSeriesItem::setPedestrianPath(const QPainterPath& scenePath) {
    prepareGeometryChange();
    pedestrianPath_ = mapFromScene(scenePath);
    update();
}
