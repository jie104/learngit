cmake_minimum_required(VERSION 2.8.3)
project(pepperl_fuchs_r2000)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -Wfatal-errors")

include_directories(include)

## Declare a cpp library
sr_library(module-laser_OMD30M
        SRCS
            src/driver/r2000_driver.cpp
            src/driver/http_command_interface.cpp
            src/driver/scan_data_receiver.cpp
            omd30m_module.cpp
        HDRS
            omd30m_module.h
        )
