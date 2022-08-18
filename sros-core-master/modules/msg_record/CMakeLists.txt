cmake_minimum_required(VERSION 2.8)
project(record_msg)

aux_source_directory(. MODULE_SRCS)


add_subdirectory(record_lib)
add_subdirectory(debug_lib)
set(MODULE_SRCS
        ${MODULE_SRCS}
        record_module.cpp record_module.h)

add_library(module-record_msg ${MODULE_SRCS})
target_link_libraries(module-record_msg record_operate debug_lib)
