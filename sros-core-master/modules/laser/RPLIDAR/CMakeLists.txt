#set(RPLIDAR_SDK_PATH, "./sdk")
#FILE(GLOB RPLIDAR_SDK_SRC
#        "${RPLIDAR_SDK_PATH}/src/*.cpp"
#        "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
#        "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
#        )
include_directories(sdk
        sdk/include
        sdk/src)

sr_library(module-laser_RPLIDAR
        SRCS
        rplidar_module.cpp
        sdk/src/arch/linux/net_serial.cpp
        sdk/src/arch/linux/net_socket.cpp
        sdk/src/arch/linux/timer.cpp
        sdk/src/hal/thread.cpp
        sdk/src/rplidar_driver.cpp
        HDRS
        rplidar_module.h
        )
