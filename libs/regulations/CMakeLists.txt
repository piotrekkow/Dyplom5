add_library(regulations
    src/regulations.cpp
)
target_include_directories(regulations 
    PUBLIC include
    PRIVATE src
)

target_link_libraries(regulations PUBLIC geometry)