#include "main_window.hpp"

#include "network/network_scene.hpp"
#include "network/network_view.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("Intersection Modeler");
    resize(1100, 700);
    scene_ = new NetworkScene(this);
    scene_->setSceneRect(-1e9, -1e9, 2e9, 2e9);
    view_ = new NetworkView(this);
    view_->setScene(scene_);
    setCentralWidget(view_);
}