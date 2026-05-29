#include "movement_item.hpp"

#include <qpainterpath.h>

#include <QPainter>

MovementItem::MovementItem(MovementId id, QPointF nodeCenter,
                           const std::vector<QPainterPath>& scenePaths)
    : id_(id) {
    setPos(nodeCenter);
    setPaths(scenePaths);
}

QPainterPath MovementItem::shape() const {
    QPainterPath p;
    for (const auto& path : paths_) p.addPath(path);
    return p;
}

void MovementItem::paint(QPainter* painter,
                         const QStyleOptionGraphicsItem* option,
                         QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(pathPen_);
    painter->drawPath(shape());
    constexpr qreal kMinScreenPx = 6.0;
    if (!shape().isEmpty() && painter->worldTransform().m11() >= kMinScreenPx) {
        QPainterPath s = shape();
        QPointF startScene = {s.elementAt(0).x, s.elementAt(0).y};
        QPointF screenPos = painter->worldTransform().map(startScene);

        painter->save();
        painter->setWorldTransform(QTransform());
        QFont font;
        font.setPointSizeF(10.0);
        painter->setFont(font);
        painter->setPen(Qt::white);
        painter->drawText(screenPos, QString::number(id_.index()));
        painter->restore();
    }
}

void MovementItem::setPaths(const std::vector<QPainterPath>& scenePaths) {
    prepareGeometryChange();
    paths_.clear();
    for (const auto& p : scenePaths) paths_.push_back(mapFromScene(p));
    recomputeBounds();
    update();
}

void MovementItem::recomputeBounds() {
    QRectF r;
    for (const auto& p : paths_) r = r.united(p.boundingRect());
    bounds_ = r.adjusted(-1.0, -1.0, 1.0, 1.0);
}