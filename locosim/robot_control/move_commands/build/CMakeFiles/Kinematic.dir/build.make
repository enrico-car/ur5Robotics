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
CMAKE_SOURCE_DIR = /home/rico/ros_ws/src/locosim/robot_control/move_commands

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rico/ros_ws/src/locosim/robot_control/move_commands/build

# Include any dependencies generated for this target.
include CMakeFiles/Kinematic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Kinematic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Kinematic.dir/flags.make

CMakeFiles/Kinematic.dir/src/kinematic.cpp.o: CMakeFiles/Kinematic.dir/flags.make
CMakeFiles/Kinematic.dir/src/kinematic.cpp.o: ../src/kinematic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rico/ros_ws/src/locosim/robot_control/move_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Kinematic.dir/src/kinematic.cpp.o"
	/usr/bin/g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kinematic.dir/src/kinematic.cpp.o -c /home/rico/ros_ws/src/locosim/robot_control/move_commands/src/kinematic.cpp

CMakeFiles/Kinematic.dir/src/kinematic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kinematic.dir/src/kinematic.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rico/ros_ws/src/locosim/robot_control/move_commands/src/kinematic.cpp > CMakeFiles/Kinematic.dir/src/kinematic.cpp.i

CMakeFiles/Kinematic.dir/src/kinematic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kinematic.dir/src/kinematic.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rico/ros_ws/src/locosim/robot_control/move_commands/src/kinematic.cpp -o CMakeFiles/Kinematic.dir/src/kinematic.cpp.s

# Object files for target Kinematic
Kinematic_OBJECTS = \
"CMakeFiles/Kinematic.dir/src/kinematic.cpp.o"

# External object files for target Kinematic
Kinematic_EXTERNAL_OBJECTS =

devel/lib/libKinematic.so: CMakeFiles/Kinematic.dir/src/kinematic.cpp.o
devel/lib/libKinematic.so: CMakeFiles/Kinematic.dir/build.make
devel/lib/libKinematic.so: CMakeFiles/Kinematic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rico/ros_ws/src/locosim/robot_control/move_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library devel/lib/libKinematic.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Kinematic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Kinematic.dir/build: devel/lib/libKinematic.so

.PHONY : CMakeFiles/Kinematic.dir/build

CMakeFiles/Kinematic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Kinematic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Kinematic.dir/clean

CMakeFiles/Kinematic.dir/depend:
	cd /home/rico/ros_ws/src/locosim/robot_control/move_commands/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rico/ros_ws/src/locosim/robot_control/move_commands /home/rico/ros_ws/src/locosim/robot_control/move_commands /home/rico/ros_ws/src/locosim/robot_control/move_commands/build /home/rico/ros_ws/src/locosim/robot_control/move_commands/build /home/rico/ros_ws/src/locosim/robot_control/move_commands/build/CMakeFiles/Kinematic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Kinematic.dir/depend

