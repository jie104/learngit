# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/clion/clion-2021.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion/clion-2021.1.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug

# Include any dependencies generated for this target.
include standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/depend.make

# Include the progress variables for this target.
include standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/progress.make

# Include the compile flags for this target's objects.
include standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/flags.make

standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.o: standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/flags.make
standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.o: ../standard_lidar_driver/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.o"
	cd /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.o -c /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/standard_lidar_driver/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp

standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.i"
	cd /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/standard_lidar_driver/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp > CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.i

standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.s"
	cd /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/standard_lidar_driver/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp -o CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.s

# Object files for target urg_c_wrapper
urg_c_wrapper_OBJECTS = \
"CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.o"

# External object files for target urg_c_wrapper
urg_c_wrapper_EXTERNAL_OBJECTS =

devel/lib/liburg_c_wrapper.so: standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp.o
devel/lib/liburg_c_wrapper.so: standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/build.make
devel/lib/liburg_c_wrapper.so: standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../devel/lib/liburg_c_wrapper.so"
	cd /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/urg_c_wrapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/build: devel/lib/liburg_c_wrapper.so

.PHONY : standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/build

standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/clean:
	cd /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver && $(CMAKE_COMMAND) -P CMakeFiles/urg_c_wrapper.dir/cmake_clean.cmake
.PHONY : standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/clean

standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/depend:
	cd /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/standard_lidar_driver /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver /home/zxj/workspace/obstacle_and_filter/standard_lidar4_ws/src/cmake-build-debug/standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : standard_lidar_driver/CMakeFiles/urg_c_wrapper.dir/depend
