add_library(road_network_lib
    src/graph.cpp
    src/demand.cpp
    src/signal.cpp

    src/util/geometry/line.cpp
    src/util/geometry/polyline.cpp
    src/util/geometry/position.cpp
    src/util/geometry/vector2.cpp

    src/derived/graph_compute.cpp
    src/derived/graph_geometry.cpp
    src/derived/demand_compute.cpp
    src/derived/signal_compute.cpp
   
    src/data/flow.cpp
    src/data/sequence.cpp
    src/data/timing.cpp
)
target_include_directories(road_network_lib 
    PUBLIC include
    PRIVATE src
)

# ── Tests ─────────────────────────────────────────────────────────────────────
# add_executable(road_network_tests

# )
# target_link_libraries(road_network_tests PRIVATE road_network_lib GTest::gtest_main)
# gtest_discover_tests(road_network_tests)