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
CMAKE_SOURCE_DIR = "/home/zxj/桌面/learngit/C++ Prime/example"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/array_form_parameter.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/array_form_parameter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/array_form_parameter.dir/flags.make

CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.o: CMakeFiles/array_form_parameter.dir/flags.make
CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.o: ../6-function/array_form_parameter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/example/6-function/array_form_parameter.cpp"

CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/example/6-function/array_form_parameter.cpp" > CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.i

CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/example/6-function/array_form_parameter.cpp" -o CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.s

# Object files for target array_form_parameter
array_form_parameter_OBJECTS = \
"CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.o"

# External object files for target array_form_parameter
array_form_parameter_EXTERNAL_OBJECTS =

array_form_parameter: CMakeFiles/array_form_parameter.dir/6-function/array_form_parameter.cpp.o
array_form_parameter: CMakeFiles/array_form_parameter.dir/build.make
array_form_parameter: CMakeFiles/array_form_parameter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable array_form_parameter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/array_form_parameter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/array_form_parameter.dir/build: array_form_parameter

.PHONY : CMakeFiles/array_form_parameter.dir/build

CMakeFiles/array_form_parameter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/array_form_parameter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/array_form_parameter.dir/clean

CMakeFiles/array_form_parameter.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/example" "/home/zxj/桌面/learngit/C++ Prime/example" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles/array_form_parameter.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/array_form_parameter.dir/depend

