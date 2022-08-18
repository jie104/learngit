#SET(SROS_DML_DEV ON)

set(LIB_DML_ALG_NAME dml-alg)

IF (CMAKE_CROSSCOMPILING)
    # 区分nxp核心板库文件
    if (${CHIP_VERSION} MATCHES "nxp")
        set(LIB_DML_ALG_NAME ${LIB_DML_ALG_NAME}-vc300-imx)
    else()
        set(LIB_DML_ALG_NAME ${LIB_DML_ALG_NAME}-vc300)
    endif()

    set(OPENCV_LIB
            opencv_core
            opencv_imgproc
            opencv_videoio
            opencv_calib3d
            opencv_objdetect
            opencv_features2d
            opencv_imgcodecs
#            opencv_highgui
            )
ELSE()
    set(OPENCV_LIB
            /usr/local/lib/libopencv_core.so.3.3.0
            /usr/local/lib/libopencv_imgproc.so.3.3.0
            /usr/local/lib/libopencv_videoio.so.3.3.0
            /usr/local/lib/libopencv_imgcodecs.so.3.3.0
            )
ENDIF()


add_library(module-vision STATIC vision_module.h vision_module.cpp)

IF (SROS_DML_DEV)
    add_subdirectory(lib-src)

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
        target_link_libraries(module-vision
                ${LIB_DML_ALG_NAME}
                ${OPENCV_LIB}
                libpng12.so
                )
    else()
        target_link_libraries(module-vision
                ${LIB_DML_ALG_NAME}
                ${OPENCV_LIB}
                png
                )
    endif()

ELSE()

    set(LIB_DML_ALG ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${LIB_DML_ALG_NAME}.a)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
        target_link_libraries(module-vision
                ${LIB_DML_ALG}
                ${LIB_FM_ALG}
                ${OPENCV_LIB}
                libpng12.so
                )
    else()
        target_link_libraries(module-vision
                ${LIB_DML_ALG}
                ${LIB_FM_ALG}
                ${OPENCV_LIB}
                png
                )
    endif()
ENDIF()

