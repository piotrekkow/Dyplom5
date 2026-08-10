#pragma once

#include <QWidget>

struct UiCore;
class RoadNetworkScene;
class RoadNetworkView;
class SceneSync;

class RoadNetworkPanel : public QWidget {
    Q_OBJECT
   public:
    explicit RoadNetworkPanel(UiCore& core, QWidget* parent = nullptr);

   private:
    UiCore& core_;
    RoadNetworkScene* scene_;
    RoadNetworkView* view_;
    SceneSync* sync_;
};