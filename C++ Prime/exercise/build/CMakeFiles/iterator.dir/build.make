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
include CMakeFiles/iterator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/iterator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/iterator.dir/flags.make

CMakeFiles/iterator.dir/iterator.cpp.o: CMakeFiles/iterator.dir/flags.make
CMakeFiles/iterator.dir/iterator.cpp.o: ../iterator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/iterator.dir/iterator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/iterator.dir/iterator.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/exercise/iterator.cpp"

CMakeFiles/iterator.dir/iterator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/iterator.dir/iterator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/exercise/iterator.cpp" > CMakeFiles/iterator.dir/iterator.cpp.i

CMakeFiles/iterator.dir/iterator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/iterator.dir/iterator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/exercise/iterator.cpp" -o CMakeFiles/iterator.dir/iterator.cpp.s

# Object files for target iterator
iterator_OBJECTS = \
"CMakeFiles/iterator.dir/iterator.cpp.o"

# External object files for target iterator
iterator_EXTERNAL_OBJECTS =

iterator: CMakeFiles/iterator.dir/iterator.cpp.o
iterator: CMakeFiles/iterator.dir/build.make
iterator: CMakeFiles/iterator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable iterator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/iterator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/iterator.dir/build: iterator

.PHONY : CMakeFiles/iterator.dir/build

CMakeFiles/iterator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/iterator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/iterator.dir/clean

CMakeFiles/iterator.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/exercise/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise/build" "/home/zxj/桌面/learngit/C++ Prime/exercise/build" "/home/zxj/桌面/learngit/C++ Prime/exercise/build/CMakeFiles/iterator.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/iterator.dir/depend

