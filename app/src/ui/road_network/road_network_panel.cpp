#include "road_network_panel.hpp"

#include <QVBoxLayout>

// temp
#include <QCoreApplication>
#include <QGraphicsPixmapItem>
#include <QPixmap>

#include "scene/road_network_scene.hpp"
#include "scene/scene_sync.hpp"
#include "view/road_network_view.hpp"

RoadNetworkPanel::RoadNetworkPanel(UiCore& core, QWidget* parent)
    : QWidget(parent), core_(core) {
    scene_ = new RoadNetworkScene(core_, this);
    scene_->setSceneRect(-1e9, -1e9, 2e9, 2e9);
    view_ = new RoadNetworkView(this);
    view_->setScene(scene_);
    sync_ = new SceneSync(core_, scene_->registry(), this);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(view_);

    // TEMP
    QPixmap px(QCoreApplication::applicationDirPath() +
               "/../../app/assets/Lazurowa-Czluchowska.jpg");

    auto pxit = scene_->addPixmap(px);
    pxit->setScale(0.041667);
    pxit->setPos({-100, -70});
    pxit->setZValue(-99);
    pxit->setOpacity(0.1);
    // TEMP
}