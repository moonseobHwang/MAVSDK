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
CMAKE_SOURCE_DIR = /home/aa6271235/Documents/DDevelop/MAVSDK/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity

# Include any dependencies generated for this target.
include offboard_velocity/CMakeFiles/offboard.dir/depend.make

# Include the progress variables for this target.
include offboard_velocity/CMakeFiles/offboard.dir/progress.make

# Include the compile flags for this target's objects.
include offboard_velocity/CMakeFiles/offboard.dir/flags.make

offboard_velocity/CMakeFiles/offboard.dir/offboard_velocity.cpp.o: offboard_velocity/CMakeFiles/offboard.dir/flags.make
offboard_velocity/CMakeFiles/offboard.dir/offboard_velocity.cpp.o: offboard_velocity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object offboard_velocity/CMakeFiles/offboard.dir/offboard_velocity.cpp.o"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/offboard.dir/offboard_velocity.cpp.o -c /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity.cpp

offboard_velocity/CMakeFiles/offboard.dir/offboard_velocity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/offboard.dir/offboard_velocity.cpp.i"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity.cpp > CMakeFiles/offboard.dir/offboard_velocity.cpp.i

offboard_velocity/CMakeFiles/offboard.dir/offboard_velocity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/offboard.dir/offboard_velocity.cpp.s"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity.cpp -o CMakeFiles/offboard.dir/offboard_velocity.cpp.s

# Object files for target offboard
offboard_OBJECTS = \
"CMakeFiles/offboard.dir/offboard_velocity.cpp.o"

# External object files for target offboard
offboard_EXTERNAL_OBJECTS =

offboard_velocity/offboard: offboard_velocity/CMakeFiles/offboard.dir/offboard_velocity.cpp.o
offboard_velocity/offboard: offboard_velocity/CMakeFiles/offboard.dir/build.make
offboard_velocity/offboard: /usr/local/lib/libmavsdk_action.so.0.35.0
offboard_velocity/offboard: /usr/local/lib/libmavsdk_offboard.so.0.35.0
offboard_velocity/offboard: /usr/local/lib/libmavsdk_telemetry.so.0.35.0
offboard_velocity/offboard: /usr/local/lib/libmavsdk.so.0.35.0
offboard_velocity/offboard: offboard_velocity/CMakeFiles/offboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable offboard"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/offboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
offboard_velocity/CMakeFiles/offboard.dir/build: offboard_velocity/offboard

.PHONY : offboard_velocity/CMakeFiles/offboard.dir/build

offboard_velocity/CMakeFiles/offboard.dir/clean:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity && $(CMAKE_COMMAND) -P CMakeFiles/offboard.dir/cmake_clean.cmake
.PHONY : offboard_velocity/CMakeFiles/offboard.dir/clean

offboard_velocity/CMakeFiles/offboard.dir/depend:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aa6271235/Documents/DDevelop/MAVSDK/examples /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/offboard_velocity/CMakeFiles/offboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : offboard_velocity/CMakeFiles/offboard.dir/depend

