cmake_minimum_required(VERSION 2.8)
project(module-laser)

add_subdirectory(OMD30M)
add_subdirectory(LMS1xx)
add_subdirectory(UST10LX)
add_subdirectory(UTM30LX)
add_subdirectory(UAM05LP)
add_subdirectory(LMS5xx)
add_subdirectory(TIM5xx)
add_subdirectory(SIM_LASER)
add_subdirectory(GAZEBOLASER)
add_subdirectory(MICRO_SCAN3)
add_subdirectory(scan_preprocess)
add_subdirectory(GENERAL)
sr_library(module-laser
        SRCS
            standard_laser_module.cpp
            laser_module_factory.cpp
        HDRS
            standard_laser_module.h
            base_laser_module.h
            laser_module_factory.h
        DEPENDS
            module-laser_UAM05LP
            module-laser_UTM30LX
            module-laser_UST10LX
            module-laser_LMS1xx
            module-laser_OMD30M
            module-laser_LMS5xx
            module-laser_TIM5xx
            module-laser_SIM
            module-laser_GAZEBO
            module-laser_MICROSCAN3
            scan_preprocess
            module-laser_GENERAL
        )

