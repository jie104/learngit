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
include CMakeFiles/copy_assigment_destroyed.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/copy_assigment_destroyed.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/copy_assigment_destroyed.dir/flags.make

CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.o: CMakeFiles/copy_assigment_destroyed.dir/flags.make
CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.o: ../13-copy_control/copy_assigment_destroyed.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.o -c "/home/zxj/learngit/C++ Prime/example/13-copy_control/copy_assigment_destroyed.cpp"

CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/learngit/C++ Prime/example/13-copy_control/copy_assigment_destroyed.cpp" > CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.i

CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/learngit/C++ Prime/example/13-copy_control/copy_assigment_destroyed.cpp" -o CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.s

# Object files for target copy_assigment_destroyed
copy_assigment_destroyed_OBJECTS = \
"CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.o"

# External object files for target copy_assigment_destroyed
copy_assigment_destroyed_EXTERNAL_OBJECTS =

copy_assigment_destroyed: CMakeFiles/copy_assigment_destroyed.dir/13-copy_control/copy_assigment_destroyed.cpp.o
copy_assigment_destroyed: CMakeFiles/copy_assigment_destroyed.dir/build.make
copy_assigment_destroyed: CMakeFiles/copy_assigment_destroyed.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable copy_assigment_destroyed"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/copy_assigment_destroyed.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/copy_assigment_destroyed.dir/build: copy_assigment_destroyed

.PHONY : CMakeFiles/copy_assigment_destroyed.dir/build

CMakeFiles/copy_assigment_destroyed.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/copy_assigment_destroyed.dir/cmake_clean.cmake
.PHONY : CMakeFiles/copy_assigment_destroyed.dir/clean

CMakeFiles/copy_assigment_destroyed.dir/depend:
	cd "/home/zxj/learngit/C++ Prime/example/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/learngit/C++ Prime/example" "/home/zxj/learngit/C++ Prime/example" "/home/zxj/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles/copy_assigment_destroyed.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/copy_assigment_destroyed.dir/depend

