cmake_minimum_required(VERSION 2.8)

project(proto C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#include(FindProtobuf)
#find_package(Protobuf REQUIRED)
#include_directories(${PROTOBUF_INCLUDE_DIR})
set(PROTOBUF_PROTOC_EXECUTABLE protoc)

sr_library(proto_tmp
        SRCS
            frame.cpp
        HDRS
            frame.h
        )

sr_proto_library(proto_main
        SRCS
            main.proto
            monitor.proto
        )

add_dependencies(proto_tmp proto_main)

sr_combined_library(core_proto
        SRCS
            proto_main
            proto_tmp
        )
