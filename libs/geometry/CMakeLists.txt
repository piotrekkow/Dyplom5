add_library(geometry
    src/line.cpp
    src/polyline.cpp
    src/position.cpp
    src/vector2.cpp
)
target_include_directories(geometry 
    PUBLIC include
    PRIVATE src
)

target_link_libraries(geometry PUBLIC common)