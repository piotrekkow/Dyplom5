#include "exit_item.hpp"

#include <QPainter>

namespace {
const QColor kBase{176, 84, 68};
}  // namespace

QRectF ExitItem::boundingRect() const {
    return {-kHitMargin, -kHitMargin, kArrowLength + 2 * kHitMargin,
            width_ + 2 * kHitMargin};
}

QPainterPath ExitItem::shape() const {
    QPainterPath path;
    path.moveTo({kArrowLength + kHitMargin, 0});
    path.lineTo(0, 0);
    path.lineTo({0, width_ + kHitMargin});

    QPainterPathStroker stroker;
    stroker.setWidth(2 * kHitMargin);

    return stroker.createStroke(path);
}

void ExitItem::paint(QPainter* p, const QStyleOptionGraphicsItem* /*option*/,
                     QWidget* /*widget*/) {
    paintBar(p, kBase);
    paintArrow(p, kBase, 0);
}
