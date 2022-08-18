cmake_minimum_required(VERSION 2.8.3)
project(laser_lms511)


include_directories(
	include
)


## Add cmake target dependencies of the executable
## same as for the library above
sr_library(module-laser_TIM5xx
		SRCS
		src/sick_tim_common.cpp
		src/sick_tim_common_tcp.cpp
		src/sick_tim551_2050001_parser.cpp
		src/abstract_parser.cpp
		tim5xx_module.cpp

		HDRS
		tim5xx_module.h
		)

