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
include CMakeFiles/IO_string.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/IO_string.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/IO_string.dir/flags.make

CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.o: CMakeFiles/IO_string.dir/flags.make
CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.o: ../8-IO_library/IO_string.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.o -c "/home/zxj/桌面/learngit/C++ Prime/exercise/8-IO_library/IO_string.cpp"

CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/learngit/C++ Prime/exercise/8-IO_library/IO_string.cpp" > CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.i

CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/learngit/C++ Prime/exercise/8-IO_library/IO_string.cpp" -o CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.s

# Object files for target IO_string
IO_string_OBJECTS = \
"CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.o"

# External object files for target IO_string
IO_string_EXTERNAL_OBJECTS =

IO_string: CMakeFiles/IO_string.dir/8-IO_library/IO_string.cpp.o
IO_string: CMakeFiles/IO_string.dir/build.make
IO_string: CMakeFiles/IO_string.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable IO_string"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IO_string.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/IO_string.dir/build: IO_string

.PHONY : CMakeFiles/IO_string.dir/build

CMakeFiles/IO_string.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/IO_string.dir/cmake_clean.cmake
.PHONY : CMakeFiles/IO_string.dir/clean

CMakeFiles/IO_string.dir/depend:
	cd "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise" "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug" "/home/zxj/桌面/learngit/C++ Prime/exercise/cmake-build-debug/CMakeFiles/IO_string.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/IO_string.dir/depend

