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
include CMakeFiles/iterator1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/iterator1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/iterator1.dir/flags.make

CMakeFiles/iterator1.dir/iterator1.cpp.o: CMakeFiles/iterator1.dir/flags.make
CMakeFiles/iterator1.dir/iterator1.cpp.o: ../iterator1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/iterator1.dir/iterator1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/iterator1.dir/iterator1.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/exercise/iterator1.cpp"

CMakeFiles/iterator1.dir/iterator1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/iterator1.dir/iterator1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/exercise/iterator1.cpp" > CMakeFiles/iterator1.dir/iterator1.cpp.i

CMakeFiles/iterator1.dir/iterator1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/iterator1.dir/iterator1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/exercise/iterator1.cpp" -o CMakeFiles/iterator1.dir/iterator1.cpp.s

# Object files for target iterator1
iterator1_OBJECTS = \
"CMakeFiles/iterator1.dir/iterator1.cpp.o"

# External object files for target iterator1
iterator1_EXTERNAL_OBJECTS =

iterator1: CMakeFiles/iterator1.dir/iterator1.cpp.o
iterator1: CMakeFiles/iterator1.dir/build.make
iterator1: CMakeFiles/iterator1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable iterator1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/iterator1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/iterator1.dir/build: iterator1

.PHONY : CMakeFiles/iterator1.dir/build

CMakeFiles/iterator1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/iterator1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/iterator1.dir/clean

CMakeFiles/iterator1.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/exercise/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise/build" "/home/zxj/桌面/learngit/C++ Prime/exercise/build" "/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles/iterator1.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/iterator1.dir/depend

