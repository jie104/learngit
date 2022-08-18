INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_SYSTEM_PROCESSOR arm)

set(SDK_COMPILER_PATH /data/toradex/sdk-v2.8/sysroots/x86_64-angstromsdk-linux/usr/bin/arm-angstrom-linux-gnueabi)

set(SDK_SYS_ROOT_PATH /data/toradex/sdk-v2.8/sysroots/armv7at2hf-neon-angstrom-linux-gnueabi)

# specify the cross compiler
SET(CMAKE_C_COMPILER ${SDK_COMPILER_PATH}/arm-angstrom-linux-gnueabi-gcc)
SET(CMAKE_CXX_COMPILER ${SDK_COMPILER_PATH}/arm-angstrom-linux-gnueabi-g++)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH ${SDK_SYS_ROOT_PATH})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=neon --sysroot=${SDK_SYS_ROOT_PATH}")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O4")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=neon -std=c++14 --sysroot=${SDK_SYS_ROOT_PATH}")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O4")


include_directories(${SDK_SYS_ROOT_PATH}/usr/include)
include_directories(${SDK_SYS_ROOT_PATH}/usr/include/c++/7.3.0)
include_directories(${SDK_SYS_ROOT_PATH}/usr/include/c++/7.3.0/arm-angstrom-linux-gnueabi)

link_directories(${SDK_SYS_ROOT_PATH}/lib)
link_directories(${SDK_SYS_ROOT_PATH}/usr/lib)
link_directories(${SDK_SYS_ROOT_PATH}/usr/lib/arm-angstrom-linux-gnueabi/7.3.0)

set(BOOST_ROOT ${SDK_SYS_ROOT_PATH}/usr)

