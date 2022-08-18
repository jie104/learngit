cmake_minimum_required(VERSION 2.8)
project(schedule)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(MODULE_FILES
        schedule_module.h
        schedule_module.cpp
    )

add_library(module-schedule ${MODULE_FILES})


