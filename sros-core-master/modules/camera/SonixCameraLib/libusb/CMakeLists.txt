
#project(omnidirAPI)

cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS " -pthread " )

add_subdirectory(os)

aux_source_directory(. DIR_SRCS)
aux_source_directory(./os OS_DIR_SRCS)

#link_directories("/usr/local/lib")

add_library(sr_libusb ${DIR_SRCS} ${OS_DIR_SRCS})



