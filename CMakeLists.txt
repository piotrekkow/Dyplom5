cmake_minimum_required(VERSION 3.16)
project(Dyplom6 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23) # std::move_only_function, std::expected
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTest REQUIRED)
enable_testing()
include(GoogleTest)

add_subdirectory(libs/common)
add_subdirectory(libs/control)
add_subdirectory(libs/geometry)
add_subdirectory(libs/regulations)
add_subdirectory(libs/road_network)
add_subdirectory(libs/signal_optimizer)
add_subdirectory(app)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(Dyplom6 PRIVATE
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