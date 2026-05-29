#include "node_item.hpp"

#include <QPainter>
#include <QVector2D>

namespace {
void drawCross(QPainter* painter, QPointF pos, qreal crossSize) {
    qreal half = crossSize / 2.0;
    QPointF pp(half, half);
    QPointF pm(half, -half);
    painter->drawLine(pos - pp, pos + pp);
    painter->drawLine(pos - pm, pos + pm);
}
}  // namespace

NodeItem::NodeItem(NodeId id, QPointF center) : id_(id) {
    setPos(center);
    recomputeBounds();
    setZValue(2);
}

QPainterPath NodeItem::shape() const {
    QPainterPath p;
    p.addEllipse(QPointF{}, handleRadius_, handleRadius_);
    return p;
}

void NodeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                     QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(handlePen_);
    painter->setBrush(handleBrush_);
    painter->drawPath(shape());

    painter->setPen(conflictPen_);
    for (const auto& p : conflictPoints_)
        drawCross(painter, p, conflictPointSize_);
}

void NodeItem::recomputeBounds() {
    QRectF r(-handleRadius_, -handleRadius_, 2 * handleRadius_,
             2 * handleRadius_);
    for (const QPointF& p : conflictPoints_) {
        QPointF dp(conflictPointSize_, conflictPointSize_);
        r = r.united(QRectF(p - dp, p + dp));
    }
    bounds_ = r.adjusted(-1.0, -1.0, 1.0, 1.0);
}

void NodeItem::setConflictPoints(const std::vector<QPointF>& scenePoints) {
    prepareGeometryChange();
    conflictPoints_.clear();
    for (const auto& p : scenePoints)
        conflictPoints_.push_back(mapFromScene(p));
    recomputeBounds();
    update();
}