cmake_minimum_required(VERSION 3.10)
project(robotworld)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_STANDARD 11)

find_package(wxWidgets REQUIRED net gl core base)
include(${wxWidgets_USE_FILE})

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
link_directories(${Boost_LIBRARY_DIR})

add_subdirectory(src)
