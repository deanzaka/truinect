# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/deanzaka/Github/truinect/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/deanzaka/Github/truinect/build

# Include any dependencies generated for this target.
include pcl_tracking/CMakeFiles/pcl_input.dir/depend.make

# Include the progress variables for this target.
include pcl_tracking/CMakeFiles/pcl_input.dir/progress.make

# Include the compile flags for this target's objects.
include pcl_tracking/CMakeFiles/pcl_input.dir/flags.make

pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o: pcl_tracking/CMakeFiles/pcl_input.dir/flags.make
pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o: /home/deanzaka/Github/truinect/src/pcl_tracking/src/pcl_input.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/deanzaka/Github/truinect/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o"
	cd /home/deanzaka/Github/truinect/build/pcl_tracking && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o -c /home/deanzaka/Github/truinect/src/pcl_tracking/src/pcl_input.cpp

pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl_input.dir/src/pcl_input.cpp.i"
	cd /home/deanzaka/Github/truinect/build/pcl_tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/deanzaka/Github/truinect/src/pcl_tracking/src/pcl_input.cpp > CMakeFiles/pcl_input.dir/src/pcl_input.cpp.i

pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl_input.dir/src/pcl_input.cpp.s"
	cd /home/deanzaka/Github/truinect/build/pcl_tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/deanzaka/Github/truinect/src/pcl_tracking/src/pcl_input.cpp -o CMakeFiles/pcl_input.dir/src/pcl_input.cpp.s

pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.requires:
.PHONY : pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.requires

pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.provides: pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.requires
	$(MAKE) -f pcl_tracking/CMakeFiles/pcl_input.dir/build.make pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.provides.build
.PHONY : pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.provides

pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.provides.build: pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o

# Object files for target pcl_input
pcl_input_OBJECTS = \
"CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o"

# External object files for target pcl_input
pcl_input_EXTERNAL_OBJECTS =

/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: pcl_tracking/CMakeFiles/pcl_input.dir/build.make
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libpcl_ros_filters.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libpcl_ros_io.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libpcl_ros_tf.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_common.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_kdtree.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_octree.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_search.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_surface.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_sample_consensus.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_filters.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_features.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_segmentation.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_io.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_registration.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_keypoints.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_recognition.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_visualization.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_people.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_outofcore.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_tracking.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libpcl_apps.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libOpenNI.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libvtkCommon.so.5.8.0
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libvtkRendering.so.5.8.0
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libvtkHybrid.so.5.8.0
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libvtkCharts.so.5.8.0
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libnodeletlib.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libbondcpp.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libclass_loader.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/libPocoFoundation.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libdl.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libroslib.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/librosbag.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/librosbag_storage.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libroslz4.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libtopic_tools.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libtf.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libtf2_ros.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libactionlib.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libmessage_filters.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libtf2.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libroscpp.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/librosconsole.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/liblog4cxx.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/librostime.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /opt/ros/indigo/lib/libcpp_common.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input: pcl_tracking/CMakeFiles/pcl_input.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input"
	cd /home/deanzaka/Github/truinect/build/pcl_tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl_input.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pcl_tracking/CMakeFiles/pcl_input.dir/build: /home/deanzaka/Github/truinect/devel/lib/pcl_tracking/pcl_input
.PHONY : pcl_tracking/CMakeFiles/pcl_input.dir/build

pcl_tracking/CMakeFiles/pcl_input.dir/requires: pcl_tracking/CMakeFiles/pcl_input.dir/src/pcl_input.cpp.o.requires
.PHONY : pcl_tracking/CMakeFiles/pcl_input.dir/requires

pcl_tracking/CMakeFiles/pcl_input.dir/clean:
	cd /home/deanzaka/Github/truinect/build/pcl_tracking && $(CMAKE_COMMAND) -P CMakeFiles/pcl_input.dir/cmake_clean.cmake
.PHONY : pcl_tracking/CMakeFiles/pcl_input.dir/clean

pcl_tracking/CMakeFiles/pcl_input.dir/depend:
	cd /home/deanzaka/Github/truinect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/deanzaka/Github/truinect/src /home/deanzaka/Github/truinect/src/pcl_tracking /home/deanzaka/Github/truinect/build /home/deanzaka/Github/truinect/build/pcl_tracking /home/deanzaka/Github/truinect/build/pcl_tracking/CMakeFiles/pcl_input.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pcl_tracking/CMakeFiles/pcl_input.dir/depend

