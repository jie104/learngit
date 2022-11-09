#指明构建项目工程所需的最低CMake版本
cmake_minimum_required(VERSION 3.10)
#指出工程名字
project(Example)

#寻找第三方库，第一个参数是库名，第二个参数REQUIRED表明该库是必须的
find_package(glog REQUIRED)

#匹配所有的C++源文件，并将其存放在SRC_FILES中
file(GLOB SRC_FILES
    "{PROJECT_SOURCE_DIR}/src/*.h"
    "{PROJECT_SOURCE_DIR}/src/*.cpp"
    "{PROJECT_SOURCE_DIR}/src/*.c"
    "{PROJECT_SOURCE_DIR}/src/*.cc"
   )

#构建可执行文件,CMAKE_PROJECT_NAME是宏，后面会自动替换为工程名Example
#源文件生成工程名Example
add_executable(${CMAKE_PROJECT_NAME} ${SRC_FILES})
#表明可执行文件由main.cpp编译而成

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE glog)

#如果使用了C++17的语法，需要打开对C++17的支持
target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_17)
#配置环节：Ctrl+Shift+p 
#构建环节：F7 或 cmake build

