cmake_minimum_required(VERSION 2.8)
project(hins_he_driver)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_BUILD_TYPE Debug )
if( CMAKE_BUILD_TYPE STREQUAL "Release")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -fPIC")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -fPIC")
endif()


file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
sensor_msgs
std_srvs
tf
message_generation
)

#add_message_files(FILES hins_srv.msg)
add_service_files(
  FILES
  hins_srv.srv
)
#generate_messages()
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime 
)


#set(lib_files )


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

# install executables and/or libraries
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
DESTINATION "${CMAKE_INSTALL_PREFIX}" COMPONENT header)

# install files
foreach(dir launch)
install(DIRECTORY ${dir}/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

