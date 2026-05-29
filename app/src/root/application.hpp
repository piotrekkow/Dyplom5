#pragma once

#include <memory>

#include "bus/network_event_bus.hpp"
#include "document.hpp"
#include "editor_context.hpp"
#include "ui/main_window.hpp"
#include "ui/network/network_scene_populator.hpp"

class Application {
   public:
    Application();
    ~Application();

    void show();

   private:
    NetworkEventBus networkBus_;
    std::unique_ptr<Document> document_;
    std::unique_ptr<EditorContext> editorCtx_;
    std::unique_ptr<MainWindow> window_;
    std::unique_ptr<NetworkScenePopulator> populator_;
};