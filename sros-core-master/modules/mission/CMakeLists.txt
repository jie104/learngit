cmake_minimum_required(VERSION 2.8)
project(mission)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(MODULE_FILES
    mission_module.h
    mission_module.cpp
    )

add_library(module-mission ${MODULE_FILES})


