# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/tahlia/cw2_redo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tahlia/cw2_redo/build

# Include any dependencies generated for this target.
include comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/depend.make

# Include the progress variables for this target.
include comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/progress.make

# Include the compile flags for this target's objects.
include comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/flags.make

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/flags.make
comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o: /home/tahlia/cw2_redo/src/comp313p/stdr_simulator/stdr_server/src/map_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tahlia/cw2_redo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o"
	cd /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o -c /home/tahlia/cw2_redo/src/comp313p/stdr_simulator/stdr_server/src/map_server.cpp

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stdr_map_server.dir/src/map_server.cpp.i"
	cd /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tahlia/cw2_redo/src/comp313p/stdr_simulator/stdr_server/src/map_server.cpp > CMakeFiles/stdr_map_server.dir/src/map_server.cpp.i

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stdr_map_server.dir/src/map_server.cpp.s"
	cd /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tahlia/cw2_redo/src/comp313p/stdr_simulator/stdr_server/src/map_server.cpp -o CMakeFiles/stdr_map_server.dir/src/map_server.cpp.s

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.requires:

.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.requires

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.provides: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.requires
	$(MAKE) -f comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/build.make comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.provides.build
.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.provides

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.provides.build: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o


# Object files for target stdr_map_server
stdr_map_server_OBJECTS = \
"CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o"

# External object files for target stdr_map_server
stdr_map_server_EXTERNAL_OBJECTS =

/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/build.make
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /home/tahlia/cw2_redo/devel/lib/libstdr_map_loader.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libtf.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libtf2_ros.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libactionlib.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libnodeletlib.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libbondcpp.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libclass_loader.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/libPocoFoundation.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libroslib.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librospack.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libmap_server_image_loader.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libroscpp.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librosconsole.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libtf2.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librostime.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libtf.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libtf2_ros.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libactionlib.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libnodeletlib.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libbondcpp.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libclass_loader.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/libPocoFoundation.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libroslib.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librospack.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libmap_server_image_loader.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libroscpp.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librosconsole.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libtf2.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/librostime.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tahlia/cw2_redo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so"
	cd /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stdr_map_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/build: /home/tahlia/cw2_redo/devel/lib/libstdr_map_server.so

.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/build

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/requires: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/src/map_server.cpp.o.requires

.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/requires

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/clean:
	cd /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server && $(CMAKE_COMMAND) -P CMakeFiles/stdr_map_server.dir/cmake_clean.cmake
.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/clean

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/depend:
	cd /home/tahlia/cw2_redo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tahlia/cw2_redo/src /home/tahlia/cw2_redo/src/comp313p/stdr_simulator/stdr_server /home/tahlia/cw2_redo/build /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server /home/tahlia/cw2_redo/build/comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_map_server.dir/depend

