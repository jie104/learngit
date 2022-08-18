cmake_minimum_required(VERSION 2.8.3)
project(laser_lms511)
add_definitions(-w)


## Add cmake target dependencies of the executable
## same as for the library above
sr_library(module-laser_LMS5xx
		SRCS
		LMS5xx.cpp
		lms5xx_module.cpp

		HDRS
		lms5xx_module.h
		LMS5xx.h
		lms_buffer.h
		lms_structs.h
		)

