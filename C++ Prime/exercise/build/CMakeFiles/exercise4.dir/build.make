# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/zxj/桌面/learngit/C++ Prime/exercise"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/zxj/桌面/learngit/C++ Prime/exercise/build"

# Include any dependencies generated for this target.
include CMakeFiles/exercise4.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/exercise4.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exercise4.dir/flags.make

CMakeFiles/exercise4.dir/exercise4.cpp.o: CMakeFiles/exercise4.dir/flags.make
CMakeFiles/exercise4.dir/exercise4.cpp.o: ../exercise4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exercise4.dir/exercise4.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exercise4.dir/exercise4.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/exercise/exercise4.cpp"

CMakeFiles/exercise4.dir/exercise4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exercise4.dir/exercise4.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/exercise/exercise4.cpp" > CMakeFiles/exercise4.dir/exercise4.cpp.i

CMakeFiles/exercise4.dir/exercise4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exercise4.dir/exercise4.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/exercise/exercise4.cpp" -o CMakeFiles/exercise4.dir/exercise4.cpp.s

# Object files for target exercise4
exercise4_OBJECTS = \
"CMakeFiles/exercise4.dir/exercise4.cpp.o"

# External object files for target exercise4
exercise4_EXTERNAL_OBJECTS =

exercise4: CMakeFiles/exercise4.dir/exercise4.cpp.o
exercise4: CMakeFiles/exercise4.dir/build.make
exercise4: CMakeFiles/exercise4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exercise4"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exercise4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exercise4.dir/build: exercise4

.PHONY : CMakeFiles/exercise4.dir/build

CMakeFiles/exercise4.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exercise4.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exercise4.dir/clean

CMakeFiles/exercise4.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/exercise/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise/build" "/home/zxj/桌面/learngit/C++ Prime/exercise/build" "/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles/exercise4.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/exercise4.dir/depend

