#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <road_network/util/ids.hpp>

class CrosswalkSeriesItem : public QGraphicsItem {
   public:
    static constexpr int Type = QGraphicsItem::UserType + 4;
    [[nodiscard]] int type() const override { return Type; }

    explicit CrosswalkSeriesItem(CrosswalkSeriesId id);
    [[nodiscard]] CrosswalkSeriesId id() const { return id_; }

    [[nodiscard]] QPainterPath shape() const override;
    [[nodiscard]] QRectF boundingRect() const override {
        return pedestrianPath_.boundingRect();
    }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;
    void setPedestrianPath(const QPainterPath& scenePath);

   private:
    CrosswalkSeriesId id_;
    QPainterPath pedestrianPath_;

    inline static const QPen pathPen_{QColor("#7b4d8084"), 0.5};
};