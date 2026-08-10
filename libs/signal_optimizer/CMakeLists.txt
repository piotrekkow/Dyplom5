add_library(signal_optimizer
    src/optimizer.cpp
    src/config_generator.cpp
    src/stage_generator.cpp
    src/stage_cover.cpp
    src/stage_sequence.cpp
    src/green_time.cpp
    src/delay.cpp
)
target_include_directories(signal_optimizer
    PUBLIC include
    PRIVATE src
)
target_link_libraries(signal_optimizer PUBLIC road_network)
