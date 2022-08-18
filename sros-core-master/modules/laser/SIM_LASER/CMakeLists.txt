cmake_minimum_required(VERSION 2.8.3)
project(laser_sim)


include_directories(
#	include
)


## Add cmake target dependencies of the executable
## same as for the library above
sr_library(module-laser_SIM
		SRCS
		simlaser_module.cpp
		network/connection.cpp
		network/simlaser_device.cpp
		HDRS
		simlaser_module.h
		network/connection.h
		network/simlaser_device.h

		)

