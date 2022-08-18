cmake_minimum_required(VERSION 2.8)
project(tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(EXECUTABLE_OUTPUT_PATH ../../bin_tools)

add_subdirectory(export_png_map)
add_subdirectory(map_json_to_map)
add_subdirectory(create_map_json)
add_subdirectory(monitor_file_to_json)
add_subdirectory(srtos_log)

link_directories(${SDK_SYS_ROOT_PATH}/usr/lib/mvs/armhf)
add_subdirectory(mvs_grap_image)

add_subdirectory(test)

