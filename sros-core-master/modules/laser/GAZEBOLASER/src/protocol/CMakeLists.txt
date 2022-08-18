cmake_minimum_required(VERSION 2.8)
project(src_sdk_network_protocol CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        all_msg.h
        src_protocol.h
        base_msg.hpp
        state_msg.hpp
        command_msg.hpp
        velocity_msg.hpp
        path_msg.hpp
        pose_msg.hpp
        parameter_msg.hpp
        signal_msg.hpp
        info_msg.hpp
        gazebo_laser_scan.hpp
        laser_scan_stamped_msg.hpp
        pose_stamped_msg.hpp
        usart_data_msg.hpp
        msg_factory.hpp
        )

add_library(protocol ${SOURCE_FILES})
set_target_properties(protocol PROPERTIES LINKER_LANGUAGE CXX)
