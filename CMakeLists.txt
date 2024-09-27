cmake_minimum_required(VERSION 3.12)

project(tutorial_project)

find_library(UAVCAN_LIB uavcan REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -std=c++11")

add_executable(node src/node_example.cpp)
target_link_libraries(node ${UAVCAN_LIB} rt)