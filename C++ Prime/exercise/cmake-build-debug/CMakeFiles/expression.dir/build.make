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
CMAKE_SOURCE_DIR = "/home/zxj/桌面/learngit/C++ Prime/exercise"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/expression.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/expression.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/expression.dir/flags.make

CMakeFiles/expression.dir/expression.cpp.o: CMakeFiles/expression.dir/flags.make
CMakeFiles/expression.dir/expression.cpp.o: ../expression.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/expression.dir/expression.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/expression.dir/expression.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/exercise/expression.cpp"

CMakeFiles/expression.dir/expression.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/expression.dir/expression.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/exercise/expression.cpp" > CMakeFiles/expression.dir/expression.cpp.i

CMakeFiles/expression.dir/expression.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/expression.dir/expression.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/exercise/expression.cpp" -o CMakeFiles/expression.dir/expression.cpp.s

# Object files for target expression
expression_OBJECTS = \
"CMakeFiles/expression.dir/expression.cpp.o"

# External object files for target expression
expression_EXTERNAL_OBJECTS =

expression: CMakeFiles/expression.dir/expression.cpp.o
expression: CMakeFiles/expression.dir/build.make
expression: CMakeFiles/expression.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable expression"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/expression.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/expression.dir/build: expression

.PHONY : CMakeFiles/expression.dir/build

CMakeFiles/expression.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/expression.dir/cmake_clean.cmake
.PHONY : CMakeFiles/expression.dir/clean

CMakeFiles/expression.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug/CMakeFiles/expression.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/expression.dir/depend

