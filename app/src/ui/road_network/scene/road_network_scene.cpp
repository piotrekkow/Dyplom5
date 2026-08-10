#include "road_network_scene.hpp"

RoadNetworkScene::RoadNetworkScene(UiCore& core, QObject* parent)
    : QGraphicsScene(parent), core_(core), registry_(*this) {}