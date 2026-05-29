#include "signal_group_item.hpp"

#include <QPainter>

SignalGroupItem::SignalGroupItem(SignalGroupId id, QPointF pos, QString label)
    : id_(id), label_(std::move(label)) {
    setPos(pos);
    setZValue(10.0);
    bounds_ = QRectF(-50.0, -20.0, 100.0, 40.0);
}

void SignalGroupItem::paint(QPainter* painter,
                            const QStyleOptionGraphicsItem* option,
                            QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    constexpr qreal kMinScreenPx = 4.0;
    if (painter->worldTransform().m11() < kMinScreenPx) return;

    QFont font("monospace", -1);
    font.setPointSizeF(kScreenPt_);
    font.setBold(true);
    QFontMetricsF fm(font);

    qreal tw = fm.horizontalAdvance(label_);
    qreal th = fm.height();
    QPointF screenOrigin = painter->worldTransform().map(QPointF(0.0, 0.0));

    painter->save();
    painter->setWorldTransform(QTransform());

    QRectF bgRect(screenOrigin.x() - tw / 2.0 - kPaddingPx_,
                  screenOrigin.y() - th / 2.0 - kPaddingPx_,
                  tw + 2.0 * kPaddingPx_, th + 2.0 * kPaddingPx_);
    painter->fillRect(bgRect, QColor(0, 0, 0, 180));
    painter->setFont(font);
    painter->setPen(Qt::white);
    painter->drawText(bgRect, Qt::AlignCenter, label_);

    painter->restore();
}
