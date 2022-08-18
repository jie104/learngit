cmake_minimum_required(VERSION 2.8)
project(lib-navigation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(LIB_NAV_SRC_FILES
        src/display.cpp
        src/heap.cpp
        src/jps_grid.cpp
        src/navigation.cpp
        src/neighbors.cpp
        src/path.cpp
        src/geometry.cpp
        src/map_erode.cpp
        src/net_navigation.cpp
        include/display.h
        include/heap.h
        include/jps_grid.h
        include/navigation.h
        include/neighbors.h
        include/path.h
        include/geometry.h
        include/map_erode.h
        include/net_navigation.h
        )

add_library(lib-navigation ${LIB_NAV_SRC_FILES})
