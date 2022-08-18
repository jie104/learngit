INCLUDE(CMakeForceCompiler)
#目标机target所在的操作系统名称
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_SYSTEM_PROCESSOR arm)

# 增加变量区分核心板(tk1,nxp)
set(CHIP_VERSION nxp)

#set(SDK_COMPILER_PATH /home/selfstyle/workspace/nxp_sdk/sysroots/x86_64-tdxsdk-linux/usr/bin/aarch64-tdx-linux)

#set(SDK_SYS_ROOT_PATH /home/selfstyle/workspace/nxp_sdk/sysroots/aarch64-tdx-linux)

set(SDK_COMPILER_PATH  /opt/tdx-xwayland/5.4.0/sysroots/x86_64-tdxsdk-linux/usr/bin/aarch64-tdx-linux)
set(SDK_SYS_ROOT_PATH  /opt/tdx-xwayland/5.4.0/sysroots/aarch64-tdx-linux)

#指定编译工具，一定要设置
# specify the cross compiler
CMAKE_FORCE_C_COMPILER(${SDK_COMPILER_PATH}/aarch64-tdx-linux-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(${SDK_COMPILER_PATH}/aarch64-tdx-linux-g++ GNU)

#指定交叉编译环境安装目录
# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH ${SDK_SYS_ROOT_PATH}  )

#从来不在指定目录下查找工具程序
# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
#只在指定目录下查找库文件
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#只在指定目录下查找头文件
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(CMAKE_C_FLAGS " -fstack-protector-strong  -D_FORTIFY_SOURCE=1 -Wformat -Wformat-security -Werror=format-security --sysroot=${SDK_SYS_ROOT_PATH}")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O1")

SET(CMAKE_CXX_FLAGS "-fstack-protector-strong  -D_FORTIFY_SOURCE=1 -Wformat -Wformat-security -Werror=format-security  -std=c++14 --sysroot=${SDK_SYS_ROOT_PATH}")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O1")

include_directories(${SDK_SYS_ROOT_PATH}/usr/include)
include_directories(${SDK_SYS_ROOT_PATH}/usr/include/c++/9.3.0)
include_directories(${SDK_SYS_ROOT_PATH}/usr/include/c++/9.3.0/aarch64-tdx-linux)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cmake/include/imx8)
set(BOOST_ROOT ${SDK_SYS_ROOT_PATH}/usr)

