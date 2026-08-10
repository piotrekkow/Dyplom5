#pragma once
#include <QMainWindow>

#include "core/ui_core.hpp"

class Project;
class RoadNetworkPanel;

class MainWindow : public QMainWindow {
    Q_OBJECT
   public:
    explicit MainWindow(Project& project, QWidget* parent = nullptr);

   private:
    UiCore core_;
    RoadNetworkPanel* roadNetworkPanel_;
};