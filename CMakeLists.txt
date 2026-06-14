cmake_minimum_required(VERSION 3.16)
project(Dyplom5 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23) # std::move_only_function, std::expected
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find_package(GTest REQUIRED)
# enable_testing()
# include(GoogleTest)

add_subdirectory(libs/road_network)
add_subdirectory(libs/signal_optimizer)
add_subdirectory(libs/fsm_simulator)
add_subdirectory(app)

if(MINGW AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(signal_optimizer_lib PUBLIC stdc++exp)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(Dyplom5 PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wsign-conversion
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Woverloaded-virtual
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
    )
endif()