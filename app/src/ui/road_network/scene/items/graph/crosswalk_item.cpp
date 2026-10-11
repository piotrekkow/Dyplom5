#include "crosswalk_item.hpp"

#include <QPainter>
#include <QPainterPathStroker>
#include <QTransform>

#include "../z_bands.hpp"

namespace {
const QColor kBase{225, 225, 225, 150};
constexpr qreal kMarkLength = 0.5;
constexpr qreal kMarkPeriod = 2 * kMarkLength;
constexpr qreal kEdgeWidth = kMarkLength;
constexpr qreal kHitMargin = 0.5;
}  // namespace

CrosswalkItem::CrosswalkItem(CrosswalkId id) : id_(id) {
    setZValue(z_band::kCrosswalks);
}

void CrosswalkItem::setGeometry(QLineF anchor, QLineF split, QLineF edge) {
    prepareGeometryChange();
    anchor_ = anchor;
    edge_ = edge;
    body_ = QPolygonF{anchor.p1(), anchor.p2(), split.p2(), split.p1()};

    const qreal len = anchor.length();
    drawEdge_ = false;
    drawSplitOffset_ = false;
    if (len > 1e-9) {
        const QPointF u = (anchor.p2() - anchor.p1()) / len;
        const QPointF n{-u.y(), u.x()};
        const auto out = [&](QPointF p) {
            return QPointF::dotProduct(p - anchor.p1(), n);
        };
        const qreal side = out(edge.p1()) < 0 ? -1.0 : 1.0;
        const qreal overhang = side * (out(edge.p1()) - out(split.p1()));
        drawEdge_ = overhang > 1e-9;
        drawSplitOffset_ = overhang > 1.0;
        splitOffset_ = split.translated(side * 1.5 * kMarkLength * n.x(),
                                        side * 1.5 * kMarkLength * n.y());
    }

    QPainterPath hit;
    hit.addPolygon(body_);
    hit.closeSubpath();
    QPainterPathStroker stroker;
    stroker.setWidth(kEdgeWidth + 2 * kHitMargin);
    if (drawEdge_) {
        QPainterPath edgePath;
        edgePath.moveTo(edge.p1());
        edgePath.lineTo(edge.p2());
        hit = hit.united(stroker.createStroke(edgePath));
    }
    if (drawSplitOffset_) {
        QPainterPath offsetPath;
        offsetPath.moveTo(splitOffset_.p1());
        offsetPath.lineTo(splitOffset_.p2());
        hit = hit.united(stroker.createStroke(offsetPath));
    }
    hitShape_ = hit;
    bounds_ = hitShape_.boundingRect();
    update();
}

void CrosswalkItem::paint(QPainter* p,
                          const QStyleOptionGraphicsItem* /*option*/,
                          QWidget* /*widget*/) {
    paintStripes(p, kBase);
    if (!drawEdge_) return;

    QPen pen{kBase, kEdgeWidth, Qt::CustomDashLine, Qt::FlatCap};
    pen.setDashPattern({1.0, 1.0});
    p->setPen(pen);
    p->setBrush(Qt::NoBrush);
    p->drawLine(edge_);
    if (drawSplitOffset_) p->drawLine(splitOffset_);
}

void CrosswalkItem::paintStripes(QPainter* p, const QColor& color) const {
    const qreal len = anchor_.length();
    if (len < 1e-9) return;

    const QPointF u = (anchor_.p2() - anchor_.p1()) / len;
    const QTransform frame{
        u.x(), u.y(), -u.y(), u.x(), anchor_.p1().x(), anchor_.p1().y()};

    QPainterPath clip;
    clip.addPolygon(body_);
    clip.closeSubpath();

    p->save();
    p->setClipPath(clip);
    p->setTransform(frame, /*combine=*/true);

    const QRectF local = frame.inverted().map(body_).boundingRect();
    const qreal x0 = std::floor(local.left() / kMarkPeriod) * kMarkPeriod;
    p->setPen(Qt::NoPen);
    p->setBrush(color);
    for (qreal x = x0; x < local.right(); x += kMarkPeriod)
        p->drawRect(QRectF{x, local.top(), kMarkLength, local.height()});
    p->restore();
}
