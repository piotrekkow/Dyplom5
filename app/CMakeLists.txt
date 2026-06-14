set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_PREFIX_PATH "C:/Qt/6.11.1/msvc2022_64")

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

add_executable(Dyplom5
    src/main.cpp

    src/root/application.cpp
    # src/root/document.cpp
    src/root/editor_context.cpp

    src/root/editor/graph_editor.cpp
    src/root/editor/demand_editor.cpp
    src/root/editor/signal_editor.cpp

    src/root/use_cases/graph_use_cases.cpp
    src/root/use_cases/demand_use_cases.cpp
    src/root/use_cases/signal_use_cases.cpp

    src/signal_debug.cpp

    src/ui/main_window.cpp
    src/ui/coord_transform.cpp
    src/ui/network/network_scene.cpp
    src/ui/network/network_view.cpp
    src/ui/network/network_scene_populator.cpp
    
    src/ui/network/items/node_item.cpp
    src/ui/network/items/entry_item.cpp
    src/ui/network/items/exit_item.cpp
    src/ui/network/items/movement_item.cpp
    src/ui/network/items/crosswalk_series_item.cpp
    src/ui/network/items/crosswalk_item.cpp
    src/ui/network/items/signal_group_item.cpp
)

target_include_directories(Dyplom5 PRIVATE src)
target_link_libraries(Dyplom5 PRIVATE
    road_network_lib
    signal_optimizer_lib
    fsm_simulator_lib
    Qt6::Core Qt6::Gui Qt6::Widgets
)

if(WIN32)
    # Find the windeployqt tool based on the Qt6 location
    get_target_property(_qmake_executable Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_executable}" DIRECTORY)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt_bin_dir}")

    # Run windeployqt after the build finishes
    add_custom_command(TARGET Dyplom5 POST_BUILD
        COMMAND "${WINDEPLOYQT_EXECUTABLE}" 
                --no-translations     # Optional: skips copying huge translation files to keep build fast
                "$<TARGET_FILE:Dyplom5>"
        COMMENT "Running windeployqt to copy DLLs..."
    )
endif()

add_custom_command(TARGET Dyplom5 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:Dyplom5>/assets"
    COMMENT "Copying app assets to runtime folder"
)