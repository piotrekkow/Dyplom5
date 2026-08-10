#include "main_window.hpp"

#include "root/project.hpp"
#include "ui/road_network/road_network_panel.hpp"

MainWindow::MainWindow(Project& project, QWidget* parent)
    : QMainWindow(parent),
      core_{.doc = project.document(),
            .bus = project.bus(),
            .project = project,
            .xf = {}} {
    setWindowTitle("Dyplom");
    resize(1280, 800);

    roadNetworkPanel_ = new RoadNetworkPanel(core_, this);
    setCentralWidget(roadNetworkPanel_);
}