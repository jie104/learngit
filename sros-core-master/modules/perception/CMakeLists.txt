cmake_minimum_required(VERSION 2.8)
project(perception)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include(cmake/version.cmake)

set(SUB_MODULE_NAME module-perception_manager)
set(SUB_MODULE_DESC "Responsible for managing the perception module.")

add_subdirectory(sensor)
add_subdirectory(lib/object_detectors)
add_subdirectory(perception-sros)

add_library(${SUB_MODULE_NAME}
    perception_manager_module.cpp
    )

target_link_libraries(${SUB_MODULE_NAME} PUBLIC
    card_detector
    circle_detector
    put_space_detector
    custom_region_detector
    handcart_detector
    QRcode_detector
    QRcode_detector_global
    decards_detector
)
