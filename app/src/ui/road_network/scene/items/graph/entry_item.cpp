#include "entry_item.hpp"

#include <QPainter>

namespace {
const QColor kBase{54, 135, 81};
}  // namespace

QRectF EntryItem::boundingRect() const {
    QRectF r{-kArrowLength - kHitMargin, -kHitMargin,
             kArrowLength + 2 * kHitMargin, width_ + 2 * kHitMargin};
    return r;
}

QPainterPath EntryItem::shape() const {
    QPainterPath path;
    path.moveTo({-kArrowLength - kHitMargin, 0});
    path.lineTo(0, 0);
    path.lineTo({0, width_ + kHitMargin});

    QPainterPathStroker stroker;
    stroker.setWidth(2 * kHitMargin);

    return stroker.createStroke(path);
}

void EntryItem::paint(QPainter* p, const QStyleOptionGraphicsItem* /*option*/,
                      QWidget* /*widget*/) {
    paintBar(p, kBase);
    paintArrow(p, kBase, -kArrowLength);
}
