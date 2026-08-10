#pragma once

#include <QGraphicsScene>

#include "items/scene_item_registry.hpp"

class UiCore;

class RoadNetworkScene : public QGraphicsScene {
    Q_OBJECT
   public:
    explicit RoadNetworkScene(UiCore& core, QObject* parent = nullptr);

    [[nodiscard]] SceneItemRegistry& registry() { return registry_; }

   private:
    UiCore& core_;
    SceneItemRegistry registry_;
};