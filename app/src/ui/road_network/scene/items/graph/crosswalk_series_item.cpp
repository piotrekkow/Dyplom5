#include "crosswalk_series_item.hpp"

#include <QPainter>
#include <QPainterPathStroker>

#include "../z_bands.hpp"

namespace {
const QColor kBase{214, 170, 92, 150};
constexpr qreal kPenWidth = 0.4;
constexpr qreal kHitWidth = 2.0;  // meters of grab area around the path
}  // namespace

CrosswalkSeriesItem::CrosswalkSeriesItem(CrosswalkSeriesId id) : id_(id) {
    setZValue(z_band::kCrosswalks + 2);
}

void CrosswalkSeriesItem::setPath(const QPainterPath& scenePath) {
    prepareGeometryChange();
    path_ = scenePath;
    if (path_.isEmpty()) {
        hitShape_ = QPainterPath{};
        bounds_ = QRectF{};
    } else {
        QPainterPathStroker stroker;
        stroker.setWidth(kHitWidth);
        hitShape_ = stroker.createStroke(path_);
        bounds_ = hitShape_.boundingRect();
    }
    update();
}

void CrosswalkSeriesItem::paint(QPainter* p,
                                const QStyleOptionGraphicsItem* /*option*/,
                                QWidget* /*widget*/) {
    if (path_.isEmpty()) return;
    p->setPen(QPen{kBase, kPenWidth});
    p->setBrush(Qt::NoBrush);
    p->drawPath(path_);
}
