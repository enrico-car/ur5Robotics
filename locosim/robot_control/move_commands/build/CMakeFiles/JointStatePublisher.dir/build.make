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
include CMakeFiles/JointStatePublisher.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/JointStatePublisher.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/JointStatePublisher.dir/flags.make

CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.o: CMakeFiles/JointStatePublisher.dir/flags.make
CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.o: ../src/joint_state_publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rico/ros_ws/src/locosim/robot_control/move_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.o"
	/usr/bin/g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.o -c /home/rico/ros_ws/src/locosim/robot_control/move_commands/src/joint_state_publisher.cpp

CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rico/ros_ws/src/locosim/robot_control/move_commands/src/joint_state_publisher.cpp > CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.i

CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rico/ros_ws/src/locosim/robot_control/move_commands/src/joint_state_publisher.cpp -o CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.s

# Object files for target JointStatePublisher
JointStatePublisher_OBJECTS = \
"CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.o"

# External object files for target JointStatePublisher
JointStatePublisher_EXTERNAL_OBJECTS =

devel/lib/libJointStatePublisher.so: CMakeFiles/JointStatePublisher.dir/src/joint_state_publisher.cpp.o
devel/lib/libJointStatePublisher.so: CMakeFiles/JointStatePublisher.dir/build.make
devel/lib/libJointStatePublisher.so: CMakeFiles/JointStatePublisher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rico/ros_ws/src/locosim/robot_control/move_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library devel/lib/libJointStatePublisher.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/JointStatePublisher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/JointStatePublisher.dir/build: devel/lib/libJointStatePublisher.so

.PHONY : CMakeFiles/JointStatePublisher.dir/build

CMakeFiles/JointStatePublisher.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/JointStatePublisher.dir/cmake_clean.cmake
.PHONY : CMakeFiles/JointStatePublisher.dir/clean

CMakeFiles/JointStatePublisher.dir/depend:
	cd /home/rico/ros_ws/src/locosim/robot_control/move_commands/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rico/ros_ws/src/locosim/robot_control/move_commands /home/rico/ros_ws/src/locosim/robot_control/move_commands /home/rico/ros_ws/src/locosim/robot_control/move_commands/build /home/rico/ros_ws/src/locosim/robot_control/move_commands/build /home/rico/ros_ws/src/locosim/robot_control/move_commands/build/CMakeFiles/JointStatePublisher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/JointStatePublisher.dir/depend
