cmake_minimum_required(VERSION 2.8)
project(src-sdk)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(protocol)

sr_library(src-sdk
        SRCS
            src_sdk_v2.cpp
            src_sdk_v1.cpp
            src_sdk.cpp
            src_upgrade.cpp
            src_sdk_do.cpp
        HDRS
            src_sdk_v2.h
            src_sdk_v1.h
            src_sdk.h
            src_upgrade.h
            src_sdk_do.h
        DEPENDS
            src-protocol
            sros-core
        )

