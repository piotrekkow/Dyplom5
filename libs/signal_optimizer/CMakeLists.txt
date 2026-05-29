add_library(signal_optimizer_lib
    src/optimizer.cpp
    src/config_generator.cpp
    src/phase_generator.cpp
    src/phase_cover.cpp
    src/phase_sequence.cpp
    src/green_time.cpp
    src/delay.cpp
)
target_include_directories(signal_optimizer_lib
    PUBLIC include
    PRIVATE src
)
target_link_libraries(signal_optimizer_lib PUBLIC road_network_lib)
