#pragma once

#include <QGraphicsItem>
#include <QString>
#include <road_network/util/ids.hpp>

class SignalGroupItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType + 6;
    [[nodiscard]] int type() const override { return Type; }

    explicit SignalGroupItem(SignalGroupId id, QPointF pos, QString label);
    [[nodiscard]] SignalGroupId id() const { return id_; }

    [[nodiscard]] QRectF boundingRect() const override { return bounds_; }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;

   private:
    SignalGroupId id_;
    QString label_;
    QRectF bounds_;

    static constexpr qreal kScreenPt_ = 15.0;  // fixed screen point size
    static constexpr qreal kPaddingPx_ = 5.0;  // fixed screen pixel padding
};