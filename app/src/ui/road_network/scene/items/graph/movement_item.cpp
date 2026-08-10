#include "movement_item.hpp"

#include <QPainter>
#include <QPainterPathStroker>

#include "../z_bands.hpp"

namespace {
const QColor kBase{170, 190, 210, 140};
constexpr qreal kPenWidth = 0.6;
constexpr qreal kHitWidth = 2.5;  // meters of grab area around the path
}  // namespace

MovementItem::MovementItem(MovementId id) : id_(id) {
    setZValue(z_band::kMovements);
}

void MovementItem::setPaths(const std::vector<QPainterPath>& scenePaths) {
    path_.clear();
    for (const auto& p : scenePaths) path_.addPath(p);
    rebuildGeometry();
}

QPainterPath MovementItem::shape() const { return hitShape_; }

void MovementItem::rebuildGeometry() {
    prepareGeometryChange();

    QPainterPathStroker stroker;
    stroker.setWidth(kHitWidth);
    hitShape_ = stroker.createStroke(path_);

    bounds_ = hitShape_.boundingRect();
    update();
}

void MovementItem::paint(QPainter* p,
                         const QStyleOptionGraphicsItem* /*option*/,
                         QWidget* /*widget*/) {
    p->setPen(QPen{kBase, kPenWidth});
    p->setBrush(Qt::NoBrush);
    p->drawPath(path_);
}
