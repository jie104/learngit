cmake_minimum_required(VERSION 3.5)
project(srosbag)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(SOURCE_FILES
        bagmodule/bag.cpp bagmodule/global_bag_config.cpp bagmodule/bag_play.cpp bagmodule/bag_record.cpp bag_manager_singleton.cpp
        msg/pub_msg/core/util/time.cpp)
set(HEAD_FILES
        bagmodule/bag.h bagmodule/global_bag_config.h bagmodule/bag_play.h bagmodule/bag_record.h
        msg/record_msg/all_msg.h msg/record_msg/base_msg.hpp msg/record_msg/gps_msg.hpp msg/record_msg/laser_scan_stamped_msg.hpp msg/record_msg/msg_factory.hpp
        msg/record_msg/pose_stamped_msg.hpp msg/record_msg/slam_info_msg.hpp bagmodule/play_option.hpp bag_manager_singleton.h)

add_library(sros_bag STATIC ${SOURCE_FILES})
target_link_libraries(sros_bag)


#add_executable(srosbag_node ${SOURCE_FILES} main.cpp)
#target_link_libraries(srosbag_node
#        glog
#        sros_bag)

SET(INSTALL_INCLUDE_PATH /usr/local/include/${PROJECT_NAME})
SET(INSTALL_LIB_PATH /usr/local/lib)
INSTALL(DIRECTORY msg DESTINATION ${INSTALL_INCLUDE_PATH})
INSTALL(DIRECTORY bagmodule DESTINATION ${INSTALL_INCLUDE_PATH})
INSTALL(TARGETS sros_bag
        LIBRARY DESTINATION ${INSTALL_LIB_PATH}
        ARCHIVE DESTINATION ${INSTALL_LIB_PATH})