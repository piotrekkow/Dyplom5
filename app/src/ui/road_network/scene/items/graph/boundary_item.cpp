#include "boundary_item.hpp"

#include <QPainter>

#include "../z_bands.hpp"

namespace {
constexpr qreal kBarWidth = 0.2;
constexpr qreal kArrowPenWidth = 0.5;
constexpr qreal kHeadLength = 1.6;
constexpr qreal kHeadHalfWidth = 0.9;
}  // namespace

BoundaryItem::BoundaryItem() { setZValue(z_band::kNodes); }

void BoundaryItem::paintBar(QPainter* p, const QColor& color) const {
    p->setPen(QPen{color, kBarWidth, Qt::DotLine});
    p->drawLine(QPointF{0, 0}, QPointF{0, width_});
}

void BoundaryItem::paintArrow(QPainter* p, const QColor& color, qreal tailX) {
    const qreal tipX = tailX + kArrowLength;
    p->setPen(QPen{color, kArrowPenWidth});
    p->drawLine(QPointF{tailX, 0}, QPointF{tipX - kHeadLength, 0});
    QPainterPath head;
    head.moveTo(tipX - kHeadLength, -kHeadHalfWidth);
    head.lineTo(tipX, 0);
    head.lineTo(tipX - kHeadLength, kHeadHalfWidth);
    head.closeSubpath();
    p->fillPath(head, color);
}