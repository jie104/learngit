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
include CMakeFiles/class_other_feature.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/class_other_feature.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/class_other_feature.dir/flags.make

CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.o: CMakeFiles/class_other_feature.dir/flags.make
CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.o: ../7-class/class_other_feature.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/example/7-class/class_other_feature.cpp"

CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/example/7-class/class_other_feature.cpp" > CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.i

CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/example/7-class/class_other_feature.cpp" -o CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.s

# Object files for target class_other_feature
class_other_feature_OBJECTS = \
"CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.o"

# External object files for target class_other_feature
class_other_feature_EXTERNAL_OBJECTS =

class_other_feature: CMakeFiles/class_other_feature.dir/7-class/class_other_feature.cpp.o
class_other_feature: CMakeFiles/class_other_feature.dir/build.make
class_other_feature: CMakeFiles/class_other_feature.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable class_other_feature"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/class_other_feature.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/class_other_feature.dir/build: class_other_feature

.PHONY : CMakeFiles/class_other_feature.dir/build

CMakeFiles/class_other_feature.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/class_other_feature.dir/cmake_clean.cmake
.PHONY : CMakeFiles/class_other_feature.dir/clean

CMakeFiles/class_other_feature.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/example" "/home/zxj/桌面/learngit/C++ Prime/example" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/example/cmake-build-debug/CMakeFiles/class_other_feature.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/class_other_feature.dir/depend

