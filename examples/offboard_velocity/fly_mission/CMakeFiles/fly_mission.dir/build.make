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
include fly_mission/CMakeFiles/fly_mission.dir/depend.make

# Include the progress variables for this target.
include fly_mission/CMakeFiles/fly_mission.dir/progress.make

# Include the compile flags for this target's objects.
include fly_mission/CMakeFiles/fly_mission.dir/flags.make

fly_mission/CMakeFiles/fly_mission.dir/fly_mission.cpp.o: fly_mission/CMakeFiles/fly_mission.dir/flags.make
fly_mission/CMakeFiles/fly_mission.dir/fly_mission.cpp.o: ../fly_mission/fly_mission.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object fly_mission/CMakeFiles/fly_mission.dir/fly_mission.cpp.o"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fly_mission.dir/fly_mission.cpp.o -c /home/aa6271235/Documents/DDevelop/MAVSDK/examples/fly_mission/fly_mission.cpp

fly_mission/CMakeFiles/fly_mission.dir/fly_mission.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fly_mission.dir/fly_mission.cpp.i"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aa6271235/Documents/DDevelop/MAVSDK/examples/fly_mission/fly_mission.cpp > CMakeFiles/fly_mission.dir/fly_mission.cpp.i

fly_mission/CMakeFiles/fly_mission.dir/fly_mission.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fly_mission.dir/fly_mission.cpp.s"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aa6271235/Documents/DDevelop/MAVSDK/examples/fly_mission/fly_mission.cpp -o CMakeFiles/fly_mission.dir/fly_mission.cpp.s

# Object files for target fly_mission
fly_mission_OBJECTS = \
"CMakeFiles/fly_mission.dir/fly_mission.cpp.o"

# External object files for target fly_mission
fly_mission_EXTERNAL_OBJECTS =

fly_mission/fly_mission: fly_mission/CMakeFiles/fly_mission.dir/fly_mission.cpp.o
fly_mission/fly_mission: fly_mission/CMakeFiles/fly_mission.dir/build.make
fly_mission/fly_mission: /usr/local/lib/libmavsdk_action.so.0.35.0
fly_mission/fly_mission: /usr/local/lib/libmavsdk_mission.so.0.35.0
fly_mission/fly_mission: /usr/local/lib/libmavsdk_telemetry.so.0.35.0
fly_mission/fly_mission: /usr/local/lib/libmavsdk.so.0.35.0
fly_mission/fly_mission: fly_mission/CMakeFiles/fly_mission.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable fly_mission"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fly_mission.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
fly_mission/CMakeFiles/fly_mission.dir/build: fly_mission/fly_mission

.PHONY : fly_mission/CMakeFiles/fly_mission.dir/build

fly_mission/CMakeFiles/fly_mission.dir/clean:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission && $(CMAKE_COMMAND) -P CMakeFiles/fly_mission.dir/cmake_clean.cmake
.PHONY : fly_mission/CMakeFiles/fly_mission.dir/clean

fly_mission/CMakeFiles/fly_mission.dir/depend:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aa6271235/Documents/DDevelop/MAVSDK/examples /home/aa6271235/Documents/DDevelop/MAVSDK/examples/fly_mission /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/fly_mission/CMakeFiles/fly_mission.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : fly_mission/CMakeFiles/fly_mission.dir/depend
