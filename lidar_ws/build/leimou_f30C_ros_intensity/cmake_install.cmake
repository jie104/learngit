<<<<<<< HEAD
# Install script for directory: /home/zxj/桌面/learngit/lidar_ws/src/leimou_f30C_ros_intensity

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxj/桌面/learngit/lidar_ws/install")
=======
# Install script for directory: /home/zxj/learngit/lidar_ws/src/leimou_f30C_ros_intensity

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxj/桌面/learngit/lidar_ws/build/leimou_f30C_ros_intensity/catkin_generated/installspace/leimou_f30_ros.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zxj/learngit/lidar_ws/build/leimou_f30C_ros_intensity/catkin_generated/installspace/leimou_f30_ros.pc")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leimou_f30_ros/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/zxj/桌面/learngit/lidar_ws/build/leimou_f30C_ros_intensity/catkin_generated/installspace/leimou_f30_rosConfig.cmake"
    "/home/zxj/桌面/learngit/lidar_ws/build/leimou_f30C_ros_intensity/catkin_generated/installspace/leimou_f30_rosConfig-version.cmake"
=======
    "/home/zxj/learngit/lidar_ws/build/leimou_f30C_ros_intensity/catkin_generated/installspace/leimou_f30_rosConfig.cmake"
    "/home/zxj/learngit/lidar_ws/build/leimou_f30C_ros_intensity/catkin_generated/installspace/leimou_f30_rosConfig-version.cmake"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leimou_f30_ros" TYPE FILE FILES "/home/zxj/桌面/learngit/lidar_ws/src/leimou_f30C_ros_intensity/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leimou_f30_ros" TYPE FILE FILES "/home/zxj/learngit/lidar_ws/src/leimou_f30C_ros_intensity/package.xml")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so"
         RPATH "")
  endif()
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/zxj/桌面/learngit/lidar_ws/devel/lib/libleimou_f30_driver.so")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/zxj/learngit/lidar_ws/devel/lib/libleimou_f30_driver.so")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libleimou_f30_driver.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node"
         RPATH "")
  endif()
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros" TYPE EXECUTABLE FILES "/home/zxj/桌面/learngit/lidar_ws/devel/lib/leimou_f30_ros/leimou_f30_node")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros" TYPE EXECUTABLE FILES "/home/zxj/learngit/lidar_ws/devel/lib/leimou_f30_ros/leimou_f30_node")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node"
<<<<<<< HEAD
         OLD_RPATH "/opt/ros/noetic/lib:/home/zxj/桌面/learngit/lidar_ws/devel/lib:"
=======
         OLD_RPATH "/opt/ros/noetic/lib:/home/zxj/learngit/lidar_ws/devel/lib:"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros/leimou_f30_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/leimou_f30_ros" TYPE DIRECTORY FILES "/home/zxj/桌面/learngit/lidar_ws/src/leimou_f30C_ros_intensity/launch")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/leimou_f30_ros" TYPE DIRECTORY FILES "/home/zxj/learngit/lidar_ws/src/leimou_f30C_ros_intensity/launch")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/leimou_f30_ros" TYPE PROGRAM FILES
<<<<<<< HEAD
    "/home/zxj/桌面/learngit/lidar_ws/src/leimou_f30C_ros_intensity/scripts/find_intelly"
    "/home/zxj/桌面/learngit/lidar_ws/src/leimou_f30C_ros_intensity/scripts/set_intelly_ip"
=======
    "/home/zxj/learngit/lidar_ws/src/leimou_f30C_ros_intensity/scripts/find_intelly"
    "/home/zxj/learngit/lidar_ws/src/leimou_f30C_ros_intensity/scripts/set_intelly_ip"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
    )
endif()

