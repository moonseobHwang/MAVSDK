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
include transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/depend.make

# Include the progress variables for this target.
include transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/progress.make

# Include the compile flags for this target's objects.
include transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/flags.make

transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.o: transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/flags.make
transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.o: ../transition_vtol_fixed_wing/transition_vtol_fixed_wing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.o"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.o -c /home/aa6271235/Documents/DDevelop/MAVSDK/examples/transition_vtol_fixed_wing/transition_vtol_fixed_wing.cpp

transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.i"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aa6271235/Documents/DDevelop/MAVSDK/examples/transition_vtol_fixed_wing/transition_vtol_fixed_wing.cpp > CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.i

transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.s"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aa6271235/Documents/DDevelop/MAVSDK/examples/transition_vtol_fixed_wing/transition_vtol_fixed_wing.cpp -o CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.s

# Object files for target transition_vtol_fixed_wing
transition_vtol_fixed_wing_OBJECTS = \
"CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.o"

# External object files for target transition_vtol_fixed_wing
transition_vtol_fixed_wing_EXTERNAL_OBJECTS =

transition_vtol_fixed_wing/transition_vtol_fixed_wing: transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/transition_vtol_fixed_wing.cpp.o
transition_vtol_fixed_wing/transition_vtol_fixed_wing: transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/build.make
transition_vtol_fixed_wing/transition_vtol_fixed_wing: /usr/local/lib/libmavsdk_action.so.0.35.0
transition_vtol_fixed_wing/transition_vtol_fixed_wing: /usr/local/lib/libmavsdk_telemetry.so.0.35.0
transition_vtol_fixed_wing/transition_vtol_fixed_wing: /usr/local/lib/libmavsdk.so.0.35.0
transition_vtol_fixed_wing/transition_vtol_fixed_wing: transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable transition_vtol_fixed_wing"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/transition_vtol_fixed_wing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/build: transition_vtol_fixed_wing/transition_vtol_fixed_wing

.PHONY : transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/build

transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/clean:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing && $(CMAKE_COMMAND) -P CMakeFiles/transition_vtol_fixed_wing.dir/cmake_clean.cmake
.PHONY : transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/clean

transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/depend:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aa6271235/Documents/DDevelop/MAVSDK/examples /home/aa6271235/Documents/DDevelop/MAVSDK/examples/transition_vtol_fixed_wing /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : transition_vtol_fixed_wing/CMakeFiles/transition_vtol_fixed_wing.dir/depend

