cmake_minimum_required(VERSION 2.8.3)

project(ord_sdk)

#set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-std=c++11)
endif()

set(SDK_SRC_DIR "src")
set(SDK_SRC
  "${SDK_SRC_DIR}/ord_driver.cpp"
  "${SDK_SRC_DIR}/lidar_address.cpp"
  "${SDK_SRC_DIR}/ord_driver_impl.cpp"
  "${SDK_SRC_DIR}/ord_driver_net.cpp"
)
SET(CMAKE_INSTALL_PREFIX <release>)

add_subdirectory(samples)

add_library(${PROJECT_NAME} STATIC ${SDK_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_include_directories(${PROJECT_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Asio/asio-1.18.2/include"
)
add_definitions(-DASIO_STANDALONE=1)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(SDK_LIB_DEPS pthread)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  add_definitions(-D_WIN32_WINNT=0x0501 -DNOMINMAX)
  set(SDK_LIB_DEPS WS2_32.Lib)
endif()

target_link_libraries(${PROJECT_NAME} ${SDK_LIB_DEPS})

