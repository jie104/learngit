#cmake_minimum_required(VERSION 3.3)
project(module_slam)

# SET(SROS_SLAM_DEV ON)

add_library(module-slam mapping_processor.cpp mapping_processor.h
        slam_module.cpp slam_module.h
        map_processor/map_processor_factory.cpp
        map_processor/map_processor_factory.h map_processor/pgv_map_processor.cpp
        map_processor/pgv_map_processor.h map_processor/lmk_map_processor.cpp
        map_processor/lmk_map_processor.h
        map_processor/slam_map_processor.cpp map_processor/slam_map_processor.h)

set(LIB_SLAM_ALG_NAME slam-alg)
find_package(OpenSSL REQUIRED)
# 输出系统发行版名称
execute_process(COMMAND lsb_release -cs
        OUTPUT_VARIABLE RELEASE_CODENAME
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )

if (CMAKE_CROSSCOMPILING)
    # 区分nxp核心板库文件
    if (${CHIP_VERSION} MATCHES "nxp")
        set(LIB_SLAM_ALG_NAME ${LIB_SLAM_ALG_NAME}-vc300-imx)
    else()
        set(LIB_SLAM_ALG_NAME ${LIB_SLAM_ALG_NAME}-vc300)
    endif()
elseif(${RELEASE_CODENAME} MATCHES "xenial")
    message(STATUS "S is Ubuntu 16.04 ${RELEASE_CODENAME}, linux version: ${CMAKE_SYSTEM}")
    set(LIB_SLAM_ALG_NAME ${LIB_SLAM_ALG_NAME}-1604)
elseif(${RELEASE_CODENAME} MATCHES "trusty")
    message(STATUS "OS is Ubuntu 14.04 ${RELEASE_CODENAME}, linux version: ${CMAKE_SYSTEM}")
    set(LIB_SLAM_ALG_NAME ${LIB_SLAM_ALG_NAME})
else()
    message(STATUS "unkown OS: ${RELEASE_CODENAME}")
endif()

IF (SROS_SLAM_DEV)
    # SLAM开发,使用lib中的代码进行链接
    add_subdirectory(lib-src)

    target_link_libraries(module-slam
            ${LIB_SLAM_ALG_NAME}
            boost_serialization
            )
#    add_dependencies(module-slam ${LIB_SLAM_ALG_NAME})
ELSE()
    # 直接链接libslam-alg.a
    set(LIB_SLAM_ALG ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${LIB_SLAM_ALG_NAME}.a)

    target_link_libraries(module-slam
            ${LIB_SLAM_ALG}
            boost_serialization
            )
ENDIF()

target_link_libraries(module-slam OpenSSL::SSL)


if(SROS_SLAM_DEV)

set(STDSLAM-LIB_HASH "unknow")
set(SLAM_COMMON_LIB_HASH "unknow")
set(NORMALSLAM_HASH "unknow")
set(SLAM_LIB_HASH "unknow")

include(cmake/git-hash.cmake)

get_hash(STDSLAM-LIB_HASH ${CMAKE_CURRENT_SOURCE_DIR}/stdslam-lib)
get_hash(SLAM_COMMON_LIB_HASH ${CMAKE_CURRENT_SOURCE_DIR}/stdslam-lib/slam_common_lib)
get_hash(NORMALSLAM_HASH ${CMAKE_CURRENT_SOURCE_DIR}/stdslam-lib/normalslam)
get_hash(SLAM_LIB_HASH ${CMAKE_CURRENT_SOURCE_DIR}/stdslam-lib/slam_lib)

configure_file(cmake/slam_version.in
               ${CMAKE_CURRENT_SOURCE_DIR}/lib/slam_version
               @ONLY)
endif()
