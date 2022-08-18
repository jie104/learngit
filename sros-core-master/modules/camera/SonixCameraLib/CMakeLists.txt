
#project(omnidirAPI)

cmake_minimum_required(VERSION 2.8)

#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( CMAKE_CXX_FLAGS " -pthread " )

add_subdirectory(libusb)
add_subdirectory(V4L2Cam)

aux_source_directory(. DIR_SRCS)

#link_directories("/usr/local/lib")

add_library(sr_libsonix_camera STATIC ${DIR_SRCS})

target_link_libraries(sr_libsonix_camera
        sr_libusb
        DMLV4L2Camera)



