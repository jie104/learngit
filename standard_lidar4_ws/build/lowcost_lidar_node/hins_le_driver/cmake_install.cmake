# Install script for directory: /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hins_le_driver/srv" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/srv/hins_srv.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hins_le_driver/cmake" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/build/lowcost_lidar_node/hins_le_driver/catkin_generated/installspace/hins_le_driver-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/include/hins_le_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/share/roseus/ros/hins_le_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/share/common-lisp/ros/hins_le_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/share/gennodejs/ros/hins_le_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/lib/python3/dist-packages/hins_le_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/lib/python3/dist-packages/hins_le_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/build/lowcost_lidar_node/hins_le_driver/catkin_generated/installspace/hins_le_driver.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hins_le_driver/cmake" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/build/lowcost_lidar_node/hins_le_driver/catkin_generated/installspace/hins_le_driver-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hins_le_driver/cmake" TYPE FILE FILES
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/build/lowcost_lidar_node/hins_le_driver/catkin_generated/installspace/hins_le_driverConfig.cmake"
    "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/build/lowcost_lidar_node/hins_le_driver/catkin_generated/installspace/hins_le_driverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hins_le_driver" TYPE FILE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver" TYPE EXECUTABLE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/lib/hins_le_driver/hins_le_driver")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver"
         OLD_RPATH "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/lib:/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/../../../devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver" TYPE EXECUTABLE FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/devel/lib/hins_le_driver/hins_le_driver")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver"
         OLD_RPATH "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/lib:/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/../../../devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hins_le_driver/hins_le_driver")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xheaderx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/install/include")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/install" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/include")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hins_le_driver/launch" TYPE DIRECTORY FILES "/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/lowcost_lidar_node/hins_le_driver/launch/")
endif()

