# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk

# Include any dependencies generated for this target.
include CMakeFiles/dji_sdk_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dji_sdk_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dji_sdk_node.dir/flags.make

CMakeFiles/dji_sdk_node.dir/src/main.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/main.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/main.cpp

CMakeFiles/dji_sdk_node.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/main.cpp > CMakeFiles/dji_sdk_node.dir/src/main.cpp.i

CMakeFiles/dji_sdk_node.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/main.cpp -o CMakeFiles/dji_sdk_node.dir/src/main.cpp.s

CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/main.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o: src/modules/dji_sdk_node_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_control.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_control.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_control.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o: src/modules/dji_sdk_node_services.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_services.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_services.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_services.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o: src/modules/dji_sdk_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o: src/modules/dji_sdk_node_mission_services.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_mission_services.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_mission_services.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_mission_services.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o: src/modules/dji_sdk_node_subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_subscriber.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_subscriber.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_subscriber.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o: src/modules/dji_sdk_node_publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_publisher.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_publisher.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_publisher.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o


CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o: CMakeFiles/dji_sdk_node.dir/flags.make
CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o: src/modules/dji_sdk_node_mobile_comm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o -c /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_mobile_comm.cpp

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_mobile_comm.cpp > CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.i

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_mobile_comm.cpp -o CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.s

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.requires:

.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.requires

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.provides: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.requires
	$(MAKE) -f CMakeFiles/dji_sdk_node.dir/build.make CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.provides.build
.PHONY : CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.provides

CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.provides.build: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o


# Object files for target dji_sdk_node
dji_sdk_node_OBJECTS = \
"CMakeFiles/dji_sdk_node.dir/src/main.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o" \
"CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o"

# External object files for target dji_sdk_node
dji_sdk_node_EXTERNAL_OBJECTS =

devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/main.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/build.make
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/libroscpp.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/librosconsole.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/liblog4cxx.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/librostime.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
devel/lib/dji_sdk/dji_sdk_node: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libboost_system.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libpthread.so
devel/lib/dji_sdk/dji_sdk_node: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
devel/lib/dji_sdk/dji_sdk_node: /usr/local/lib/libdjiosdk-core.a
devel/lib/dji_sdk/dji_sdk_node: CMakeFiles/dji_sdk_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable devel/lib/dji_sdk/dji_sdk_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dji_sdk_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dji_sdk_node.dir/build: devel/lib/dji_sdk/dji_sdk_node

.PHONY : CMakeFiles/dji_sdk_node.dir/build

CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/main.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_control.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_services.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mission_services.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_subscriber.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_publisher.cpp.o.requires
CMakeFiles/dji_sdk_node.dir/requires: CMakeFiles/dji_sdk_node.dir/src/modules/dji_sdk_node_mobile_comm.cpp.o.requires

.PHONY : CMakeFiles/dji_sdk_node.dir/requires

CMakeFiles/dji_sdk_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dji_sdk_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dji_sdk_node.dir/clean

CMakeFiles/dji_sdk_node.dir/depend:
	cd /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk /home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dji_sdk_node.dir/depend

