set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_PREFIX_PATH "C:/Qt/6.11.1/msvc2022_64")

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

add_executable(Dyplom6
    src/main.cpp

    src/application.cpp
    src/log/log.cpp
    src/log/control_log.cpp
    src/util/group_label.cpp
    src/root/editor.cpp

    src/root/editor/graph_editor.cpp
    src/root/editor/demand_editor.cpp
    src/root/editor/signal_editor.cpp
    src/root/editor/control_editor.cpp

    src/ui/main_window.cpp
    src/ui/core/coord_transform.cpp
    src/ui/road_network/road_network_panel.cpp
    src/ui/road_network/scene/road_network_scene.cpp
    src/ui/road_network/scene/scene_sync.cpp
    # Header-only QObject: listed so AUTOMOC finds it.
    src/ui/road_network/scene/items/scene_item_registry.hpp
    src/ui/road_network/scene/items/graph/boundary_item.cpp
    src/ui/road_network/scene/items/graph/node_item.cpp
    src/ui/road_network/scene/items/graph/entry_item.cpp
    src/ui/road_network/scene/items/graph/exit_item.cpp
    src/ui/road_network/scene/items/graph/movement_item.cpp
    src/ui/road_network/scene/items/graph/crosswalk_item.cpp
    src/ui/road_network/scene/items/graph/crosswalk_series_item.cpp
    src/ui/road_network/view/road_network_view.cpp
)

target_include_directories(Dyplom6 PRIVATE src)
target_link_libraries(Dyplom6 PUBLIC
    road_network
    signal_optimizer
    Qt6::Core Qt6::Gui Qt6::Widgets
)