#pragma once
#include <QMainWindow>

class NetworkScene;
class NetworkView;

class MainWindow : public QMainWindow {
    Q_OBJECT
   public:
    explicit MainWindow(QWidget* parent = nullptr);
    NetworkScene* scene() { return scene_; }

   private:
    NetworkScene* scene_;
    NetworkView* view_;
};