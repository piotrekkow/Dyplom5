#include "exit_item.hpp"

#include <QPainter>

ExitItem::ExitItem(ExitId id, QPointF origin, QPointF heading)
    : id_(id), heading_(heading) {
    setPos(origin);
    recomputeBounds();
}

QPainterPath ExitItem::shape() const {
    QPainterPath p;
    p.addEllipse(QPointF{}, handleRadius_, handleRadius_);
    p.moveTo(QPointF{});
    p.lineTo(heading_ * handleArrowLength_);
    return p;
}

void ExitItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                     QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(exitPen_);
    painter->drawPath(shape());

    for (const auto& p : lanePoints_)
        painter->drawEllipse(p, lanePointSize_, lanePointSize_);
}

void ExitItem::recomputeBounds() {
    QRectF r(-handleRadius_, -handleRadius_, 2 * handleRadius_,
             2 * handleRadius_);
    QPointF tip = heading_ * handleArrowLength_;
    r = r.united(QRectF(tip, tip).adjusted(-1.0, -1.0, -1.0, -1.0));
    for (const QPointF& p : lanePoints_) {
        QPointF dp(lanePointSize_, lanePointSize_);
        r = r.united(QRectF(p - dp, p + dp));
    }
    bounds_ = r.adjusted(-1.0, -1.0, 1.0, 1.0);
}

void ExitItem::setLanePoints(const std::vector<QPointF>& scenePoints) {
    prepareGeometryChange();
    lanePoints_.clear();
    for (const auto& p : scenePoints) lanePoints_.push_back(mapFromScene(p));
    recomputeBounds();
    update();
}
