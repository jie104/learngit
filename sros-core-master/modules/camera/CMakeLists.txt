cmake_minimum_required(VERSION 2.8)
project(module-camera)

# NXP暂不开放ifman相机驱动
if (${CHIP_VERSION} MATCHES "nxp")
    #add_subdirectory(camera_device/ifm3d)
else()
    add_subdirectory(camera_device/ifm3d)
endif()

if(CMAKE_CROSSCOMPILING)
    set(OPENCV_LIB
#            /data/toradex/sdk-v2.8/sysroots/armv7at2hf-neon-angstrom-linux-gnueabi/usr/lib/libopencv_core.so.3.3.0
#            /data/toradex/sdk-v2.8/sysroots/armv7at2hf-neon-angstrom-linux-gnueabi/usr/lib/libopencv_imgproc.so.3.3.0
#            /data/toradex/sdk-v2.8/sysroots/armv7at2hf-neon-angstrom-linux-gnueabi/usr/lib/libopencv_imgcodecs.so.3.3.0
#            /data/toradex/sdk-v2.8/sysroots/armv7at2hf-neon-angstrom-linux-gnueabi/usr/lib/libopencv_videoio.so.3.3.0
            opencv_core
            opencv_imgproc
            opencv_videoio
            opencv_imgcodecs
            )
else()
    # 在开发PC上，由于同时存在opencv 2.4和3.3，为避免混乱，需要手动指定lib路径
    set(OPENCV_LIB
            /usr/local/lib/libopencv_core.so.3.3.0
            /usr/local/lib/libopencv_imgproc.so.3.3.0
            /usr/local/lib/libopencv_videoio.so.3.3.0
#            /usr/local/lib/libopencv_imgcodecs.so
#            /usr/local/lib/libopencv_imgcodecs.so.3.3
            /usr/local/lib/libopencv_imgcodecs.so.3.3.0
            )
endif()
add_subdirectory(SonixCameraLib)
add_subdirectory(CqUsbCam)


# NXP暂不开放ifman相机驱动
if (${CHIP_VERSION} MATCHES "nxp")
    sr_library(module-camera
        SRCS
            camera_module.cpp
            camera.cpp
            azure_camera.cpp
            d435/d435_driver.cpp
            camera_device/svc200_camera_device.cpp
            camera_device/halley_camera_device.cpp
            camera_device/halley_camera_device_manager.cpp
            Halley/halley_driver.cpp
        HDRS
            camera_module.h
            camera.h
            azure_camera.h
            camera_device/svc200_camera_device.h
            camera_device/halley_camera_device.h
            camera_device/halley_camera_device_manager.h
            Halley/halley_driver.h
        DEPENDS
            realsense2
            #ifm3d_driver
            sr_libsonix_camera
            CqUsbCam
#            realsense2
            ${OPENCV_LIB}
#            opencv_contrib
            MvCameraControl
        )
else()
    sr_library(module-camera
        SRCS
            camera_module.cpp
            camera.cpp
            azure_camera.cpp
            d435/d435_driver.cpp
            camera_device/svc200_camera_device.cpp
            camera_device/halley_camera_device.cpp
            camera_device/halley_camera_device_manager.cpp
            Halley/halley_driver.cpp
        HDRS
            camera_module.h
            camera.h
            azure_camera.h
            camera_device/svc200_camera_device.h
            camera_device/halley_camera_device.h
            camera_device/halley_camera_device_manager.h
            Halley/halley_driver.h
        DEPENDS
            realsense2
            ifm3d_driver
            sr_libsonix_camera
            CqUsbCam
    #            realsense2
            ${OPENCV_LIB}
    #            opencv_contrib
            MvCameraControl
        )
endif()
