# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/kir/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kir/catkin_ws/build

# Include any dependencies generated for this target.
include snake_control2/CMakeFiles/snake_test.dir/depend.make

# Include the progress variables for this target.
include snake_control2/CMakeFiles/snake_test.dir/progress.make

# Include the compile flags for this target's objects.
include snake_control2/CMakeFiles/snake_test.dir/flags.make

snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o: snake_control2/CMakeFiles/snake_test.dir/flags.make
snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o: /home/kir/catkin_ws/src/snake_control2/src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kir/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o"
	cd /home/kir/catkin_ws/build/snake_control2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/snake_test.dir/src/test.cpp.o -c /home/kir/catkin_ws/src/snake_control2/src/test.cpp

snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/snake_test.dir/src/test.cpp.i"
	cd /home/kir/catkin_ws/build/snake_control2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kir/catkin_ws/src/snake_control2/src/test.cpp > CMakeFiles/snake_test.dir/src/test.cpp.i

snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/snake_test.dir/src/test.cpp.s"
	cd /home/kir/catkin_ws/build/snake_control2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kir/catkin_ws/src/snake_control2/src/test.cpp -o CMakeFiles/snake_test.dir/src/test.cpp.s

snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.requires:

.PHONY : snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.requires

snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.provides: snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.requires
	$(MAKE) -f snake_control2/CMakeFiles/snake_test.dir/build.make snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.provides.build
.PHONY : snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.provides

snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.provides.build: snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o


# Object files for target snake_test
snake_test_OBJECTS = \
"CMakeFiles/snake_test.dir/src/test.cpp.o"

# External object files for target snake_test
snake_test_EXTERNAL_OBJECTS =

/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: snake_control2/CMakeFiles/snake_test.dir/build.make
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/libroscpp.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/librosconsole.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/librostime.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /opt/ros/melodic/lib/libcpp_common.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/kir/catkin_ws/devel/lib/snake_control2/snake_test: snake_control2/CMakeFiles/snake_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kir/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/kir/catkin_ws/devel/lib/snake_control2/snake_test"
	cd /home/kir/catkin_ws/build/snake_control2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/snake_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
snake_control2/CMakeFiles/snake_test.dir/build: /home/kir/catkin_ws/devel/lib/snake_control2/snake_test

.PHONY : snake_control2/CMakeFiles/snake_test.dir/build

snake_control2/CMakeFiles/snake_test.dir/requires: snake_control2/CMakeFiles/snake_test.dir/src/test.cpp.o.requires

.PHONY : snake_control2/CMakeFiles/snake_test.dir/requires

snake_control2/CMakeFiles/snake_test.dir/clean:
	cd /home/kir/catkin_ws/build/snake_control2 && $(CMAKE_COMMAND) -P CMakeFiles/snake_test.dir/cmake_clean.cmake
.PHONY : snake_control2/CMakeFiles/snake_test.dir/clean

snake_control2/CMakeFiles/snake_test.dir/depend:
	cd /home/kir/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kir/catkin_ws/src /home/kir/catkin_ws/src/snake_control2 /home/kir/catkin_ws/build /home/kir/catkin_ws/build/snake_control2 /home/kir/catkin_ws/build/snake_control2/CMakeFiles/snake_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : snake_control2/CMakeFiles/snake_test.dir/depend

