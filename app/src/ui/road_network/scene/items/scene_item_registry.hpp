#pragma once

#include <QObject>

#include "item_registry.hpp"
#include "graph/crosswalk_item.hpp"
#include "graph/crosswalk_series_item.hpp"
#include "graph/entry_item.hpp"
#include "graph/exit_item.hpp"
#include "graph/movement_item.hpp"
#include "graph/node_item.hpp"

class SceneItemRegistry : public QObject {
    Q_OBJECT
   public:
    explicit SceneItemRegistry(QGraphicsScene& scene) : scene_(scene) {}

    ItemRegistry<NodeId, NodeItem>& nodes() { return nodes_; }
    ItemRegistry<EntryId, EntryItem>& entries() { return entries_; }
    ItemRegistry<ExitId, ExitItem>& exits() { return exits_; }
    ItemRegistry<MovementId, MovementItem>& movements() { return movements_; }
    ItemRegistry<CrosswalkSeriesId, CrosswalkSeriesItem>& crosswalkSeries() {
        return crosswalkSeries_;
    }
    ItemRegistry<CrosswalkId, CrosswalkItem>& crosswalks() {
        return crosswalks_;
    }

   signals:
    void itemAdded(QGraphicsItem* item);

   private:
    QGraphicsScene& scene_;

    ItemRegistry<NodeId, NodeItem> nodes_{
        scene_, [this](QGraphicsItem* i) { emit itemAdded(i); }};
    ItemRegistry<EntryId, EntryItem> entries_{
        scene_, [this](QGraphicsItem* i) { emit itemAdded(i); }};
    ItemRegistry<ExitId, ExitItem> exits_{
        scene_, [this](QGraphicsItem* i) { emit itemAdded(i); }};
    ItemRegistry<MovementId, MovementItem> movements_{
        scene_, [this](QGraphicsItem* i) { emit itemAdded(i); }};
    ItemRegistry<CrosswalkSeriesId, CrosswalkSeriesItem> crosswalkSeries_{
        scene_, [this](QGraphicsItem* i) { emit itemAdded(i); }};
    ItemRegistry<CrosswalkId, CrosswalkItem> crosswalks_{
        scene_, [this](QGraphicsItem* i) { emit itemAdded(i); }};
};
