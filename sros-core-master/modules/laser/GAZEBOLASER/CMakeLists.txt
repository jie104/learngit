cmake_minimum_required(VERSION 2.8.3)
project(laser_gazebo)

add_subdirectory(src)

include_directories(
#	include
)


## Add cmake target dependencies of the executable
## same as for the library above
sr_library(module-laser_GAZEBO
		SRCS
			gazebo_laser.cpp

		HDRS
			gazebo_laser.h
		DEPENDS
			gazebo_laser_device

		)

