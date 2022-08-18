INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_SYSTEM_PROCESSOR arm)

# 增加变量区分核心板(tk1,nxp)
set(CHIP_VERSION tk1)

set(SDK_COMPILER_PATH /data/toradex/sdk/sysroots/x86_64-angstromsdk-linux/usr/bin/arm-angstrom-linux-gnueabi)

set(SDK_SYS_ROOT_PATH /data/toradex/sdk/sysroots/armv7at2hf-vfp-neon-angstrom-linux-gnueabi)

# specify the cross compiler
CMAKE_FORCE_C_COMPILER(${SDK_COMPILER_PATH}/arm-angstrom-linux-gnueabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(${SDK_COMPILER_PATH}/arm-angstrom-linux-gnueabi-g++ GNU)

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
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O4")


include_directories(${SDK_SYS_ROOT_PATH}/usr/include)
include_directories(${SDK_SYS_ROOT_PATH}/usr/include/c++/5.2.1)
include_directories(${SDK_SYS_ROOT_PATH}/usr/include/c++/5.2.1/arm-angstrom-linux-gnueabi)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cmake/include/tk1)
set(BOOST_ROOT ${SDK_SYS_ROOT_PATH}/usr)

