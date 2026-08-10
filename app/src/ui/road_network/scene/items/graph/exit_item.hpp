#pragma once

#include <road_network/util/ids.hpp>

#include "boundary_item.hpp"

class ExitItem : public BoundaryItem {
   public:
    explicit ExitItem(ExitId id) : id_(id) {}

    [[nodiscard]] ExitId id() const { return id_; }

    [[nodiscard]] QRectF boundingRect() const override;
    [[nodiscard]] QPainterPath shape() const override;
    void paint(QPainter* p, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

   private:
    ExitId id_;
};
