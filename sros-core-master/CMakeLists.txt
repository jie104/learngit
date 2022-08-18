cmake_minimum_required(VERSION 2.8)

project(sros)

if (POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#################################################
# 处理cmake文件包含

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(functions)

#include(cpplint)

#################################################
# 设置版本号

#include_directories(/home/lfc/proto_3/include/)
#link_directories(/home/lfc/proto_3/lib/)

set(SROS_MAJOR_VERSION 4)
set(SROS_MINOR_VERSION 19)
set(SROS_PATCH_VERSION 0)
set(SROS_IS_ALPHA_VERSION FALSE)

set(SROS_VERSION ${SROS_MAJOR_VERSION}.${SROS_MINOR_VERSION}.${SROS_PATCH_VERSION})

set(SROS_ALPHA_VERSION_STR "")
if (SROS_IS_ALPHA_VERSION)
    set(SROS_ALPHA_VERSION_STR "-alpha-1")
endif ()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/version.sh.in
        ${CMAKE_CURRENT_SOURCE_DIR}/version.sh)

#################################################
# 设置编译信息

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -DNDEBUG")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("SROS is too slow to be useful in debug mode.")
else ()
    message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

#################################################
# 功能开关

# 是否启用视觉功能（需要依赖libopencv/librealsense等）
option(ENABLE_VISION "ON for enable or OFF for diable" OFF)
option(ENABLE_SROSBAG "ON for enable or OFF for diable" ON)

if (CMAKE_CROSSCOMPILING)
    set(ENABLE_VISION ON)
else ()
    set(ENABLE_VISION ON) # 本地编译不编译视觉模块
endif ()

if (ENABLE_VISION)
    add_definitions(-DENABLE_VISION)
endif ()

if(ENABLE_SROSBAG)
    add_definitions(-DENABLE_SROSBAG)
endif ()

if (CMAKE_CROSSCOMPILING)
    # arm交叉编译
    add_definitions(-DRUN_ON_VEHICLE)
else ()
    add_definitions(-DRUN_ON_PC)
endif ()

#################################################
# 第三方依赖库

find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
#find_package(Protobuf REQUIRED)
#################################################
# 设置内部依赖头文件包含路径


include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/thirty-party/)

include_directories(./thirty-party/SQLiteCpp/include/)

include_directories(${SROS_SDK_PATH})
#include_directories(./modules/camera/Halley/)

#################################################
# 添加依赖模块

add_subdirectory(core)
add_subdirectory(src/sdk)
add_subdirectory(modules)

add_subdirectory(thirty-party)

add_subdirectory(tools)
#################################################

# list of script files of the SROS
set(SROS_SCRIPTS
        cpplint.py
        )
source_group(scripts FILES ${SROS_SCRIPTS})

# list of doc files of the SROS
set(SROS_DOCS
        README.md
        )
source_group(doc FILES ${SROS_DOCS})

# list of src files of the SROS
set(SROS_SRCS
        main.cpp
        )
source_group(src FILES ${SROS_SRCS})

#################################################
add_executable(sros ${SROS_SRCS} ${SROS_SCRIPTS} ${SROS_DOCS})

if(CHIP_VERSION STREQUAL "nxp")

    target_link_libraries(sros
        sros-core
        src-sdk
        sros-modules
        sros-thirty-party
        pthread
        dl
        boost_thread
        boost_system
        boost_filesystem
        boost_serialization
        glog
        gflags
        z
        # tcmalloc_minimal # 经测试用tcmalloc_and_profiler，及时没有生产heapprofile，也会导致旋转顶升处理不过来
        OpenNI2
        DepthUtils
        OniFile
        orbbec
        )

else()

    target_link_libraries(sros
        sros-core
        src-sdk
        sros-modules
        sros-thirty-party
        pthread
        dl
        boost_thread
        boost_system
        boost_filesystem
        boost_serialization
        glog
        gflags
        z
        tcmalloc_minimal
        OpenNI2
        DepthUtils
        OniFile
        orbbec
        )

endif()


################# gtest ################################
#
# 使用说明：
#   本地测试：
#       cd ~workspace/sros-core/build-native
#       cmake ..
#       make -j7 gtest_helper
#       ctest  -R gtest_helper -c debug -V
#   tk1测试：
#       cd ~workspace/sros-core/update
#       ./one.sh
#       升级完后，手动去tk1中测试

enable_testing()
add_subdirectory(test/util)
