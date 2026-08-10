#include "node_item.hpp"

#include <QPainter>

#include "../z_bands.hpp"

namespace {
const QColor kFill{140, 135, 82, 40};
const QColor kOutline{204, 204, 120, 70};
constexpr qreal kPenWidth = 2.0;
}  // namespace

NodeItem::NodeItem(NodeId id) : id_(id) { setZValue(z_band::kNodes); }

QRectF NodeItem::boundingRect() const {
    const qreal r = kRadius + kPenWidth;
    return {-r, -r, 2 * r, 2 * r};
}

QPainterPath NodeItem::shape() const {
    QPainterPath path;
    path.addEllipse(QPointF{0, 0}, kRadius, kRadius);
    return path;
}

void NodeItem::paint(QPainter* p, const QStyleOptionGraphicsItem* /*option*/,
                     QWidget* /*widget*/) {
    const QColor outline = kOutline;
    p->setPen(QPen{outline, kPenWidth});
    p->setBrush(kFill);
    p->drawEllipse(QPointF{0, 0}, kRadius, kRadius);
}
