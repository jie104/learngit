cmake_minimum_required(VERSION 2.8)
project(avoid_oba)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(AVOID_OBA_SRC_FILES
        avdoba_sample_manager.hpp
        avdoba_processor.hpp
        avdoba_module_para.hpp
        collide_region_manager.hpp
        obstacle_points_manager.hpp
        particle_info_msg.hpp
        curve_sample/base_curve_sample.hpp
        curve_sample/bezier_curve_operator.hpp
        curve_sample/bezier_curve_sample.hpp
        curve_sample/circle_curve_operator.hpp
        curve_sample/circle_curve_sample.hpp
        curve_sample/curve_sample_factory.cpp
        curve_sample/curve_sample_factory.h
        curve_sample/line_curve_operator.hpp
        curve_sample/line_curve_sample.hpp
        curve_sample/rotate_curve_operator.hpp
        curve_sample/rotate_curve_sample.hpp
        )

add_library(avoid_oba ${AVOID_OBA_SRC_FILES})
