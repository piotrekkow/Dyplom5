add_library(road_network
    src/graph.cpp
    src/demand.cpp
    src/signal.cpp
    src/control.cpp

    src/derived/compute_helpers.cpp
    src/derived/graph_compute.cpp
    src/derived/graph_geometry.cpp
    src/derived/demand_compute.cpp
    src/derived/signal_compute.cpp
    src/derived/control_compute.cpp
)
target_include_directories(road_network
    PUBLIC include
    PRIVATE src
)

target_link_libraries(road_network PUBLIC common)
target_link_libraries(road_network PUBLIC control)
target_link_libraries(road_network PUBLIC geometry)
target_link_libraries(road_network PUBLIC regulations)