#---------------------------------------------------------------------------------------
# Get livox_ros_driver version from test/livox_ros_driver.h
#---------------------------------------------------------------------------------------
file(READ "cmake/version.h" PERCEPTION_VERSION_FILE)
string(REGEX MATCH "PERCEPTION_VER_MAJOR ([0-9]+)" _  "${PERCEPTION_VERSION_FILE}")
set(ver_major ${CMAKE_MATCH_1})

string(REGEX MATCH "PERCEPTION_VER_MINOR ([0-9]+)" _  "${PERCEPTION_VERSION_FILE}")
set(ver_minor ${CMAKE_MATCH_1})

string(REGEX MATCH "PERCEPTION_VER_PATCH ([0-9]+)" _  "${PERCEPTION_VERSION_FILE}")
set(ver_patch ${CMAKE_MATCH_1})

if (NOT DEFINED ver_major OR NOT DEFINED ver_minor OR NOT DEFINED ver_patch)
    message(FATAL_ERROR "Could not extract valid version from cmake/version.h")
endif()
set (PERCEPTION_VERSION "${ver_major}.${ver_minor}.${ver_patch}")
message("${PERCEPTION_VERSION}")