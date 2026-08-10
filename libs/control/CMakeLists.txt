add_library(control
    src/timing_math.cpp
)
target_include_directories(control 
    PUBLIC include
    PRIVATE src
)
target_link_libraries(control PUBLIC regulations)