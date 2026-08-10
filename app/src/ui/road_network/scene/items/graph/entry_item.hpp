#pragma once

#include <road_network/util/ids.hpp>

#include "boundary_item.hpp"

class EntryItem : public BoundaryItem {
   public:
    explicit EntryItem(EntryId id) : id_(id) {}
    [[nodiscard]] EntryId id() const { return id_; }

    [[nodiscard]] QRectF boundingRect() const override;
    [[nodiscard]] QPainterPath shape() const override;
    void paint(QPainter* p, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

   private:
    EntryId id_;
};