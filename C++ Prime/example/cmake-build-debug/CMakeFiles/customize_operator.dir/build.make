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
CMAKE_COMMAND = /home/zxj/下载/CLion-2021.1.3/clion-2021.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zxj/下载/CLion-2021.1.3/clion-2021.1.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/zxj/learngit/C++ Prime/example"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/zxj/learngit/C++ Prime/example/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/customize_operator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/customize_operator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/customize_operator.dir/flags.make

CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.o: CMakeFiles/customize_operator.dir/flags.make
CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.o: ../10-generic_algorithms/customize_operator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.o -c "/home/zxj/learngit/C++ Prime/example/10-generic_algorithms/customize_operator.cpp"

CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/learngit/C++ Prime/example/10-generic_algorithms/customize_operator.cpp" > CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.i

CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/learngit/C++ Prime/example/10-generic_algorithms/customize_operator.cpp" -o CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.s

# Object files for target customize_operator
customize_operator_OBJECTS = \
"CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.o"

# External object files for target customize_operator
customize_operator_EXTERNAL_OBJECTS =

customize_operator: CMakeFiles/customize_operator.dir/10-generic_algorithms/customize_operator.cpp.o
customize_operator: CMakeFiles/customize_operator.dir/build.make
customize_operator: CMakeFiles/customize_operator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable customize_operator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/customize_operator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/customize_operator.dir/build: customize_operator

.PHONY : CMakeFiles/customize_operator.dir/build

CMakeFiles/customize_operator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/customize_operator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/customize_operator.dir/clean

CMakeFiles/customize_operator.dir/depend:
	cd "/home/zxj/learngit/C++ Prime/example/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/learngit/C++ Prime/example" "/home/zxj/learngit/C++ Prime/example" "/home/zxj/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles/customize_operator.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/customize_operator.dir/depend

