cmake_minimum_required(VERSION 2.8)
project(feat_recognition)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES
        rack_recognition.cpp
        rack_recognition.h

        charging_pile_recognition.cpp
        charging_pile_recognition.h

        base_recognition.cpp
        base_recognition.hpp

        fitting/circle_processor.cpp
        fitting/circle_processor.h

        fitting/line_processor.cpp
        fitting/line_processor.h

        fitting/fitting_processor.hpp

        fitting/fitting_proc_factory.cpp
        fitting/fitting_proc_factory.h

        gicp.h
        )

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
