cmake_minimum_required(VERSION 2.8)

cmake_minimum_required(VERSION 2.8.3)
project(gazebo_laser_device)

include_directories(
        #	include
)


## Add cmake target dependencies of the executable
## same as for the library above
sr_library(${PROJECT_NAME}
        SRCS
        connection.cpp
        gazebo_laser_device.cpp

        HDRS
        connection.h
        gazebo_laser_device.h

        )
