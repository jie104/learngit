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
include CMakeFiles/inbuilt_array2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/inbuilt_array2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/inbuilt_array2.dir/flags.make

CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.o: CMakeFiles/inbuilt_array2.dir/flags.make
CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.o: ../inbuilt_array2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/example/inbuilt_array2.cpp"

CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/example/inbuilt_array2.cpp" > CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.i

CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/example/inbuilt_array2.cpp" -o CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.s

# Object files for target inbuilt_array2
inbuilt_array2_OBJECTS = \
"CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.o"

# External object files for target inbuilt_array2
inbuilt_array2_EXTERNAL_OBJECTS =

inbuilt_array2: CMakeFiles/inbuilt_array2.dir/inbuilt_array2.cpp.o
inbuilt_array2: CMakeFiles/inbuilt_array2.dir/build.make
inbuilt_array2: CMakeFiles/inbuilt_array2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable inbuilt_array2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/inbuilt_array2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/inbuilt_array2.dir/build: inbuilt_array2

.PHONY : CMakeFiles/inbuilt_array2.dir/build

CMakeFiles/inbuilt_array2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/inbuilt_array2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/inbuilt_array2.dir/clean

CMakeFiles/inbuilt_array2.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/example" "/home/zxj/桌面/learngit/C++ Prime/example" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles/inbuilt_array2.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/inbuilt_array2.dir/depend

