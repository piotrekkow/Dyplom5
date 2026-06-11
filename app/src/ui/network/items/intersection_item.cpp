#include "intersection_item.hpp"

#include <QPainter>

IntersectionItem::IntersectionItem(IntersectionId id,
                                   const QPainterPath& sceneOutline)
    : id_(id) {
    setZValue(5);
    setOutline(sceneOutline);
}

void IntersectionItem::paint(QPainter* painter,
                             const QStyleOptionGraphicsItem* option,
                             QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(outlinePen_);
    painter->setBrush(fillBrush_);
    painter->drawPath(outline_);
}

void IntersectionItem::setOutline(const QPainterPath& sceneOutline) {
    prepareGeometryChange();
    outline_ = mapFromScene(sceneOutline);
    bounds_ = outline_.boundingRect().adjusted(-1.0, -1.0, 1.0, 1.0);
    update();
}
