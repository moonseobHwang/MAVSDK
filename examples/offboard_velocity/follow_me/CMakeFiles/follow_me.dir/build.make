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
include follow_me/CMakeFiles/follow_me.dir/depend.make

# Include the progress variables for this target.
include follow_me/CMakeFiles/follow_me.dir/progress.make

# Include the compile flags for this target's objects.
include follow_me/CMakeFiles/follow_me.dir/flags.make

follow_me/CMakeFiles/follow_me.dir/follow_me.cpp.o: follow_me/CMakeFiles/follow_me.dir/flags.make
follow_me/CMakeFiles/follow_me.dir/follow_me.cpp.o: ../follow_me/follow_me.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object follow_me/CMakeFiles/follow_me.dir/follow_me.cpp.o"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/follow_me.dir/follow_me.cpp.o -c /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me/follow_me.cpp

follow_me/CMakeFiles/follow_me.dir/follow_me.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/follow_me.dir/follow_me.cpp.i"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me/follow_me.cpp > CMakeFiles/follow_me.dir/follow_me.cpp.i

follow_me/CMakeFiles/follow_me.dir/follow_me.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/follow_me.dir/follow_me.cpp.s"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me/follow_me.cpp -o CMakeFiles/follow_me.dir/follow_me.cpp.s

follow_me/CMakeFiles/follow_me.dir/fake_location_provider.cpp.o: follow_me/CMakeFiles/follow_me.dir/flags.make
follow_me/CMakeFiles/follow_me.dir/fake_location_provider.cpp.o: ../follow_me/fake_location_provider.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object follow_me/CMakeFiles/follow_me.dir/fake_location_provider.cpp.o"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/follow_me.dir/fake_location_provider.cpp.o -c /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me/fake_location_provider.cpp

follow_me/CMakeFiles/follow_me.dir/fake_location_provider.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/follow_me.dir/fake_location_provider.cpp.i"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me/fake_location_provider.cpp > CMakeFiles/follow_me.dir/fake_location_provider.cpp.i

follow_me/CMakeFiles/follow_me.dir/fake_location_provider.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/follow_me.dir/fake_location_provider.cpp.s"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me/fake_location_provider.cpp -o CMakeFiles/follow_me.dir/fake_location_provider.cpp.s

# Object files for target follow_me
follow_me_OBJECTS = \
"CMakeFiles/follow_me.dir/follow_me.cpp.o" \
"CMakeFiles/follow_me.dir/fake_location_provider.cpp.o"

# External object files for target follow_me
follow_me_EXTERNAL_OBJECTS =

follow_me/follow_me: follow_me/CMakeFiles/follow_me.dir/follow_me.cpp.o
follow_me/follow_me: follow_me/CMakeFiles/follow_me.dir/fake_location_provider.cpp.o
follow_me/follow_me: follow_me/CMakeFiles/follow_me.dir/build.make
follow_me/follow_me: /usr/local/lib/libmavsdk_action.so.0.35.0
follow_me/follow_me: /usr/local/lib/libmavsdk_follow_me.so.0.35.0
follow_me/follow_me: /usr/local/lib/libmavsdk_telemetry.so.0.35.0
follow_me/follow_me: /usr/local/lib/libmavsdk.so.0.35.0
follow_me/follow_me: follow_me/CMakeFiles/follow_me.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable follow_me"
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/follow_me.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
follow_me/CMakeFiles/follow_me.dir/build: follow_me/follow_me

.PHONY : follow_me/CMakeFiles/follow_me.dir/build

follow_me/CMakeFiles/follow_me.dir/clean:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me && $(CMAKE_COMMAND) -P CMakeFiles/follow_me.dir/cmake_clean.cmake
.PHONY : follow_me/CMakeFiles/follow_me.dir/clean

follow_me/CMakeFiles/follow_me.dir/depend:
	cd /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aa6271235/Documents/DDevelop/MAVSDK/examples /home/aa6271235/Documents/DDevelop/MAVSDK/examples/follow_me /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me /home/aa6271235/Documents/DDevelop/MAVSDK/examples/offboard_velocity/follow_me/CMakeFiles/follow_me.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : follow_me/CMakeFiles/follow_me.dir/depend
