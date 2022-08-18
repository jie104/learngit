cmake_minimum_required(VERSION 2.8)
project(record_operat)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

aux_source_directory(. MODULE_SRCS)

set(MODULE_SRCS
        record_operator.h
        record_operator.cpp
        play_operator.h
        play_operator.cpp
        record_msg/base_record_msg.hpp
        record_msg/base_record_msg.cpp
        record_msg/obstacle_record_msg.hpp record_msg/base_record_msg.cpp
        record_msg/record_msg_factory.cpp record_msg/record_msg_factory.h)

add_library(record_operate ${MODULE_SRCS})
