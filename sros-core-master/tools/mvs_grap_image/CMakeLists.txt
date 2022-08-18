cmake_minimum_required(VERSION 2.8)
project(sros_mvs_grap_image)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

add_executable(sros_mvs_grap_image GrabImage.cpp)

include_directories(${SDK_SYS_ROOT_PATH}/usr/include/mvs)
#link_directories(${SDK_SYS_ROOT_PATH}/usr/lib/mvs/armhf)

IF (CMAKE_CROSSCOMPILING)
    set(LIB_DML_ALG_NAME ${LIB_DML_ALG_NAME}-vc300)
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

target_link_libraries(sros_mvs_grap_image
        pthread
        MvCameraControl
        ${OPENCV_LIB}
#        /data/toradex/sdk/sysroots/armv7at2hf-vfp-neon-angstrom-linux-gnueabi/usr/lib/mvs/armhf/libMvCameraControl.so
        )

