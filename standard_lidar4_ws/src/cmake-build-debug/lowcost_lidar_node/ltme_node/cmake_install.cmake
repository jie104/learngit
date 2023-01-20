# Install script for directory: /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ltme_node/srv" TYPE FILE FILES
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv"
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv"
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ltme_node/cmake" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/ltme_node/catkin_generated/installspace/ltme_node-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/include/ltme_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/share/roseus/ros/ltme_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/share/common-lisp/ros/ltme_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/share/gennodejs/ros/ltme_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/lib/python3/dist-packages/ltme_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/lib/python3/dist-packages/ltme_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/ltme_node/catkin_generated/installspace/ltme_node.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ltme_node/cmake" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/ltme_node/catkin_generated/installspace/ltme_node-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ltme_node/cmake" TYPE FILE FILES
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/ltme_node/catkin_generated/installspace/ltme_nodeConfig.cmake"
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/ltme_node/catkin_generated/installspace/ltme_nodeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ltme_node" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ltme_node" TYPE EXECUTABLE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/lib/ltme_node/ltme_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/ltme_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/update_firmware" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/update_firmware")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/update_firmware"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ltme_node" TYPE EXECUTABLE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/devel/lib/ltme_node/update_firmware")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/update_firmware" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/update_firmware")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ltme_node/update_firmware")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ltme_node" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/launch")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/ltme_node/lib/ldcp_sdk/cmake_install.cmake")

endif()

