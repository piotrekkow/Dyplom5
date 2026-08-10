#pragma once

#include <memory>
#include <regulations/regulations.hpp>

#include "root/project.hpp"
#include "ui/main_window.hpp"

class Application {
   public:
    Application();
    ~Application();

    void show();

   private:
    Regulations regulations_ = Regulations::defaults();
    std::unique_ptr<Project> project_;
    std::unique_ptr<MainWindow> window_;
};
