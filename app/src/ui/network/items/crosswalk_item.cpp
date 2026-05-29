#include "crosswalk_item.hpp"

#include <qpainter.h>

#include <QPainter>

CrosswalkItem::CrosswalkItem(CrosswalkId id, QLineF l1, QLineF split, QLineF l2)
    : id_(id), l1_(l1), split_(split), l2_(l2) {
    setZValue(1);
}

QPainterPath CrosswalkItem::shape() const {
    QPainterPath p;
    p.moveTo(l1_.p1());
    p.lineTo(l1_.p2());
    p.moveTo(l2_.p1());
    p.lineTo(l2_.p2());
    p.moveTo(split_.p1());
    p.lineTo(split_.p2());

    QPainterPathStroker stroker;
    stroker.setWidth(2.0);  // hit-test width

    return stroker.createStroke(p);
}

void CrosswalkItem::paint(QPainter* painter,
                          const QStyleOptionGraphicsItem* option,
                          QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setPen(line1Pen_);
    painter->drawLine(l1_);
    constexpr qreal kMinScreenPx = 6.0;
    if (!l1_.isNull() && painter->worldTransform().m11() >= kMinScreenPx) {
        QPointF mean = (l1_.p1() + l1_.p2() + l2_.p1() + l2_.p2()) / 4.0;
        QPointF screenPos = painter->worldTransform().map(mean);

        painter->save();
        painter->setWorldTransform(QTransform());
        QFont font;
        font.setPointSizeF(10.0);
        painter->setFont(font);
        painter->setPen(Qt::white);
        painter->drawText(screenPos, QString::number(id_.index()));
        painter->restore();
    }
    painter->setPen(splitPen_);
    painter->drawLine(split_);
    painter->setPen(line2Pen_);
    painter->drawLine(l2_);
}

void CrosswalkItem::setLines(QLineF l1, QLineF split, QLineF l2) {
    prepareGeometryChange();
    l1_ = l1;
    split_ = split;
    l2_ = l2;
    recomputeBounds();
    update();
}

void CrosswalkItem::recomputeBounds() {
    QRectF r;
    r = r.united(QRectF(l1_.p1(), l1_.p2()));
    r = r.united(QRectF(l2_.p1(), l2_.p2()));
    r = r.united(QRectF(split_.p1(), split_.p2()));
    r = r.adjusted(1.0, 1.0, 1.0, 1.0);
    bounds_ = r;
}