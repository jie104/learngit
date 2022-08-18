cmake_minimum_required(VERSION 2.8)
project(src_sdk_network_protocol CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        all_msg.h
        src_protocol.h
        base_msg.hpp
        state_msg.hpp
        info_msg.hpp
        command_msg.hpp
        path_msg.hpp
        pose_msg.hpp
        signal_msg.hpp
        parameter_msg.hpp
        action_state_msg.hpp
        monitor_state.hpp
        security_msg.hpp
        )

add_library(src-protocol ${SOURCE_FILES})
set_target_properties(src-protocol PROPERTIES LINKER_LANGUAGE CXX)
