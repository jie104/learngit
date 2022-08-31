<<<<<<< HEAD
# Install script for directory: /home/zxj/桌面/learngit/lidar_ws/src/scan_filter

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxj/桌面/learngit/lidar_ws/install")
=======
# Install script for directory: /home/zxj/learngit/lidar_ws/src/scan_filter

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxj/learngit/lidar_ws/install")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxj/桌面/learngit/lidar_ws/build/scan_filter/catkin_generated/installspace/scan_filter.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxj/learngit/lidar_ws/build/scan_filter/catkin_generated/installspace/scan_filter.pc")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scan_filter/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/zxj/桌面/learngit/lidar_ws/build/scan_filter/catkin_generated/installspace/scan_filterConfig.cmake"
    "/home/zxj/桌面/learngit/lidar_ws/build/scan_filter/catkin_generated/installspace/scan_filterConfig-version.cmake"
=======
    "/home/zxj/learngit/lidar_ws/build/scan_filter/catkin_generated/installspace/scan_filterConfig.cmake"
    "/home/zxj/learngit/lidar_ws/build/scan_filter/catkin_generated/installspace/scan_filterConfig-version.cmake"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scan_filter" TYPE FILE FILES "/home/zxj/桌面/learngit/lidar_ws/src/scan_filter/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scan_filter" TYPE FILE FILES "/home/zxj/learngit/lidar_ws/src/scan_filter/package.xml")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

