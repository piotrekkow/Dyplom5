#include "signal_group_item.hpp"

#include <QFontMetricsF>
#include <QPainter>

QFont SignalGroupItem::labelFont() {
    QFont font("monospace", -1);
    font.setPointSizeF(kScreenPt_);
    font.setBold(true);
    return font;
}

SignalGroupItem::SignalGroupItem(SignalGroupId id, QPointF pos, QString label)
    : id_(id), label_(std::move(label)) {
    setPos(pos);
    setZValue(10.0);
    setFlag(QGraphicsItem::ItemIgnoresTransformations);

    QFontMetricsF fm(labelFont());
    qreal tw = fm.horizontalAdvance(label_);
    qreal th = fm.height();
    bounds_ = QRectF(-tw / 2.0 - kPaddingPx_, -th / 2.0 - kPaddingPx_,
                     tw + 2.0 * kPaddingPx_, th + 2.0 * kPaddingPx_);
}

void SignalGroupItem::paint(QPainter* painter,
                            const QStyleOptionGraphicsItem* option,
                            QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->fillRect(bounds_, QColor(0, 0, 0, 180));
    painter->setFont(labelFont());
    painter->setPen(Qt::white);
    painter->drawText(bounds_, Qt::AlignCenter, label_);
}
