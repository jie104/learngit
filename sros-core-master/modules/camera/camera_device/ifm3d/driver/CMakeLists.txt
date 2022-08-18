cmake_minimum_required(VERSION 2.8.3)
project(ifm3d_driver)

include_directories(../modules/camera/include)
include_directories(../modules/image/include)
include_directories(../modules/framegrabber/include)
include_directories(../3rdParty)
#find_package(OpenCV 2.4 REQUIRED)
#file(GLOB DRIVER_SOURCES *.hpp)

add_library(ifm3d_driver ifm3d_driver.cpp ifm3d_driver.hpp)

target_link_libraries(ifm3d_driver
		ifm3d_camera_shared
		ifm3d_framegrabber_shared
		ifm3d_image_shared
		glog
  )