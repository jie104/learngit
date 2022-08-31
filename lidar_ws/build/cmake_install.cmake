<<<<<<< HEAD
# Install script for directory: /home/zxj/桌面/learngit/lidar_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zxj/桌面/learngit/lidar_ws/install")
=======
# Install script for directory: /home/zxj/learngit/lidar_ws/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/zxj/桌面/learngit/lidar_ws/install/_setup_util.py")
=======
   "/home/zxj/learngit/lidar_ws/install/_setup_util.py")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/zxj/桌面/learngit/lidar_ws/install" TYPE PROGRAM FILES "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/_setup_util.py")
=======
file(INSTALL DESTINATION "/home/zxj/learngit/lidar_ws/install" TYPE PROGRAM FILES "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/_setup_util.py")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/zxj/桌面/learngit/lidar_ws/install/env.sh")
=======
   "/home/zxj/learngit/lidar_ws/install/env.sh")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/zxj/桌面/learngit/lidar_ws/install" TYPE PROGRAM FILES "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/env.sh")
=======
file(INSTALL DESTINATION "/home/zxj/learngit/lidar_ws/install" TYPE PROGRAM FILES "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/env.sh")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/zxj/桌面/learngit/lidar_ws/install/setup.bash;/home/zxj/桌面/learngit/lidar_ws/install/local_setup.bash")
=======
   "/home/zxj/learngit/lidar_ws/install/setup.bash;/home/zxj/learngit/lidar_ws/install/local_setup.bash")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/zxj/桌面/learngit/lidar_ws/install" TYPE FILE FILES
    "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/setup.bash"
    "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/local_setup.bash"
=======
file(INSTALL DESTINATION "/home/zxj/learngit/lidar_ws/install" TYPE FILE FILES
    "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/setup.bash"
    "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/local_setup.bash"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/zxj/桌面/learngit/lidar_ws/install/setup.sh;/home/zxj/桌面/learngit/lidar_ws/install/local_setup.sh")
=======
   "/home/zxj/learngit/lidar_ws/install/setup.sh;/home/zxj/learngit/lidar_ws/install/local_setup.sh")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/zxj/桌面/learngit/lidar_ws/install" TYPE FILE FILES
    "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/setup.sh"
    "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/local_setup.sh"
=======
file(INSTALL DESTINATION "/home/zxj/learngit/lidar_ws/install" TYPE FILE FILES
    "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/setup.sh"
    "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/local_setup.sh"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/zxj/桌面/learngit/lidar_ws/install/setup.zsh;/home/zxj/桌面/learngit/lidar_ws/install/local_setup.zsh")
=======
   "/home/zxj/learngit/lidar_ws/install/setup.zsh;/home/zxj/learngit/lidar_ws/install/local_setup.zsh")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/zxj/桌面/learngit/lidar_ws/install" TYPE FILE FILES
    "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/local_setup.zsh"
=======
file(INSTALL DESTINATION "/home/zxj/learngit/lidar_ws/install" TYPE FILE FILES
    "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/local_setup.zsh"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/zxj/桌面/learngit/lidar_ws/install/.rosinstall")
=======
   "/home/zxj/learngit/lidar_ws/install/.rosinstall")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/zxj/桌面/learngit/lidar_ws/install" TYPE FILE FILES "/home/zxj/桌面/learngit/lidar_ws/build/catkin_generated/installspace/.rosinstall")
=======
file(INSTALL DESTINATION "/home/zxj/learngit/lidar_ws/install" TYPE FILE FILES "/home/zxj/learngit/lidar_ws/build/catkin_generated/installspace/.rosinstall")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
<<<<<<< HEAD
  include("/home/zxj/桌面/learngit/lidar_ws/build/gtest/cmake_install.cmake")
  include("/home/zxj/桌面/learngit/lidar_ws/build/scan_filter/cmake_install.cmake")
  include("/home/zxj/桌面/learngit/lidar_ws/build/leimou_f30C_ros_intensity/cmake_install.cmake")
  include("/home/zxj/桌面/learngit/lidar_ws/build/srosbag_pub/cmake_install.cmake")
  include("/home/zxj/桌面/learngit/lidar_ws/build/wj_716_lidar/cmake_install.cmake")
=======
  include("/home/zxj/learngit/lidar_ws/build/gtest/cmake_install.cmake")
  include("/home/zxj/learngit/lidar_ws/build/scan_filter/cmake_install.cmake")
  include("/home/zxj/learngit/lidar_ws/build/leimou_f30C_ros_intensity/cmake_install.cmake")
  include("/home/zxj/learngit/lidar_ws/build/srosbag_pub/cmake_install.cmake")
  include("/home/zxj/learngit/lidar_ws/build/standard_lidar_driver/cmake_install.cmake")
  include("/home/zxj/learngit/lidar_ws/build/wj_716_lidar/cmake_install.cmake")
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/zxj/桌面/learngit/lidar_ws/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/zxj/learngit/lidar_ws/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
