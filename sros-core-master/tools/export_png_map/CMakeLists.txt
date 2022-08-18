cmake_minimum_required(VERSION 2.8)
project(sros_export_png_map)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -lz")

add_executable(sros_export_png_map export_png_map.cpp)

# 修复tk1下无法生成png导致加载地图失败的问题
target_link_libraries(sros_export_png_map
    sros-core
    core_core
    src-sdk
    sros-modules
    sros-thirty-party
    SQLiteCpp
    sqlite3
    pthread
    dl
    boost_thread
    boost_system
    boost_filesystem
    glog
    libpng12.so
)
