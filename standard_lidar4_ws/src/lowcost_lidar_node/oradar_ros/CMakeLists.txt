cmake_minimum_required(VERSION 2.8.3)
project(oradar_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    rosconsole
    roscpp
    sensor_msgs
    message_generation
    std_msgs
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

##################################
########    Build         ########
##################################

set(ORADAR_SDK_DIR "sdk")
add_subdirectory(${ORADAR_SDK_DIR})

include_directories(
  "${ORADAR_SDK_DIR}/include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(oradar_scan
  "src/oradar_scan_node.cpp"
)

add_dependencies(oradar_scan
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(oradar_scan
  ${catkin_LIBRARIES}
  ord_sdk
)

add_executable(oradar_pointcloud
  "src/oradar_pointcloud_node.cpp"
)

add_dependencies(oradar_pointcloud
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(oradar_pointcloud
  ${catkin_LIBRARIES}
  ord_sdk
)

##################################
########    INSTALL       ########
##################################

install(TARGETS oradar_scan oradar_pointcloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
