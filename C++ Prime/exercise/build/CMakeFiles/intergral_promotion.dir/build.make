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
CMAKE_SOURCE_DIR = "/home/zxj/桌面/Study/C++ Prime/exercise"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/zxj/桌面/Study/C++ Prime/exercise/build"

# Include any dependencies generated for this target.
include CMakeFiles/intergral_promotion.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/intergral_promotion.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/intergral_promotion.dir/flags.make

CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.o: CMakeFiles/intergral_promotion.dir/flags.make
CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.o: ../intergral_promotion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zxj/桌面/Study/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.o -c "/home/zxj/桌面/Study/C++ Prime/exercise/intergral_promotion.cpp"

CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zxj/桌面/Study/C++ Prime/exercise/intergral_promotion.cpp" > CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.i

CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zxj/桌面/Study/C++ Prime/exercise/intergral_promotion.cpp" -o CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.s

# Object files for target intergral_promotion
intergral_promotion_OBJECTS = \
"CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.o"

# External object files for target intergral_promotion
intergral_promotion_EXTERNAL_OBJECTS =

intergral_promotion: CMakeFiles/intergral_promotion.dir/intergral_promotion.cpp.o
intergral_promotion: CMakeFiles/intergral_promotion.dir/build.make
intergral_promotion: CMakeFiles/intergral_promotion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zxj/桌面/Study/C++ Prime/exercise/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable intergral_promotion"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/intergral_promotion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/intergral_promotion.dir/build: intergral_promotion

.PHONY : CMakeFiles/intergral_promotion.dir/build

CMakeFiles/intergral_promotion.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/intergral_promotion.dir/cmake_clean.cmake
.PHONY : CMakeFiles/intergral_promotion.dir/clean

CMakeFiles/intergral_promotion.dir/depend:
	cd "/home/zxj/桌面/Study/C++ Prime/exercise/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zxj/桌面/Study/C++ Prime/exercise" "/home/zxj/桌面/Study/C++ Prime/exercise" "/home/zxj/桌面/Study/C++ Prime/exercise/build" "/home/zxj/桌面/Study/C++ Prime/exercise/build" "/home/zxj/桌面/Study/C++ Prime/exercise/build/CMakeFiles/intergral_promotion.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/intergral_promotion.dir/depend

