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
CMAKE_SOURCE_DIR = /home/mingh/robot-controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mingh/robot-controller/build

# Include any dependencies generated for this target.
include CMakeFiles/RobotController.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RobotController.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RobotController.dir/flags.make

CMakeFiles/RobotController.dir/RobotController.cpp.o: CMakeFiles/RobotController.dir/flags.make
CMakeFiles/RobotController.dir/RobotController.cpp.o: ../RobotController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mingh/robot-controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RobotController.dir/RobotController.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers -o CMakeFiles/RobotController.dir/RobotController.cpp.o -c /home/mingh/robot-controller/RobotController.cpp

CMakeFiles/RobotController.dir/RobotController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RobotController.dir/RobotController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers -E /home/mingh/robot-controller/RobotController.cpp > CMakeFiles/RobotController.dir/RobotController.cpp.i

CMakeFiles/RobotController.dir/RobotController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RobotController.dir/RobotController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers -S /home/mingh/robot-controller/RobotController.cpp -o CMakeFiles/RobotController.dir/RobotController.cpp.s

CMakeFiles/RobotController.dir/RobotController.cpp.o.requires:

.PHONY : CMakeFiles/RobotController.dir/RobotController.cpp.o.requires

CMakeFiles/RobotController.dir/RobotController.cpp.o.provides: CMakeFiles/RobotController.dir/RobotController.cpp.o.requires
	$(MAKE) -f CMakeFiles/RobotController.dir/build.make CMakeFiles/RobotController.dir/RobotController.cpp.o.provides.build
.PHONY : CMakeFiles/RobotController.dir/RobotController.cpp.o.provides

CMakeFiles/RobotController.dir/RobotController.cpp.o.provides.build: CMakeFiles/RobotController.dir/RobotController.cpp.o


# Object files for target RobotController
RobotController_OBJECTS = \
"CMakeFiles/RobotController.dir/RobotController.cpp.o"

# External object files for target RobotController
RobotController_EXTERNAL_OBJECTS =

RobotController: CMakeFiles/RobotController.dir/RobotController.cpp.o
RobotController: CMakeFiles/RobotController.dir/build.make
RobotController: /usr/local/lib/libvisp_robot.so.3.4.1
RobotController: /usr/local/lib/libvisp_gui.so.3.4.1
RobotController: /usr/local/lib/libvisp_vs.so.3.4.1
RobotController: /usr/local/lib/libvisp_detection.so.3.4.1
RobotController: /usr/lib/x86_64-linux-gnu/libSM.so
RobotController: /usr/lib/x86_64-linux-gnu/libICE.so
RobotController: /usr/lib/x86_64-linux-gnu/libX11.so
RobotController: /usr/lib/x86_64-linux-gnu/libXext.so
RobotController: /usr/local/lib/libvisp_sensor.so.3.4.1
RobotController: /usr/lib/x86_64-linux-gnu/libv4l2.so
RobotController: /usr/lib/x86_64-linux-gnu/libv4lconvert.so
RobotController: /usr/lib/x86_64-linux-gnu/libdc1394.so
RobotController: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
RobotController: /usr/local/lib/librealsense2.so
RobotController: /usr/local/lib/libvisp_vision.so.3.4.1
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
RobotController: /usr/local/lib/libvisp_io.so.3.4.1
RobotController: /usr/lib/x86_64-linux-gnu/libjpeg.so
RobotController: /usr/lib/x86_64-linux-gnu/libpng.so
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
RobotController: /usr/local/lib/libvisp_visual_features.so.3.4.1
RobotController: /usr/local/lib/libvisp_me.so.3.4.1
RobotController: /usr/local/lib/libvisp_blob.so.3.4.1
RobotController: /usr/local/lib/libvisp_core.so.3.4.1
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/liblapack.so
RobotController: /usr/lib/x86_64-linux-gnu/libblas.so
RobotController: /usr/lib/x86_64-linux-gnu/libxml2.so
RobotController: /usr/lib/x86_64-linux-gnu/libpthread.so
RobotController: /usr/lib/gcc/x86_64-linux-gnu/7/libgomp.so
RobotController: /usr/lib/x86_64-linux-gnu/libpthread.so
RobotController: /usr/lib/gcc/x86_64-linux-gnu/7/libgomp.so
RobotController: /usr/lib/x86_64-linux-gnu/libz.so
RobotController: /usr/lib/x86_64-linux-gnu/libzbar.so
RobotController: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
RobotController: /usr/lib/x86_64-linux-gnu/libm.so
RobotController: /usr/lib/x86_64-linux-gnu/libnsl.so
RobotController: CMakeFiles/RobotController.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mingh/robot-controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable RobotController"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RobotController.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RobotController.dir/build: RobotController

.PHONY : CMakeFiles/RobotController.dir/build

CMakeFiles/RobotController.dir/requires: CMakeFiles/RobotController.dir/RobotController.cpp.o.requires

.PHONY : CMakeFiles/RobotController.dir/requires

CMakeFiles/RobotController.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RobotController.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RobotController.dir/clean

CMakeFiles/RobotController.dir/depend:
	cd /home/mingh/robot-controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mingh/robot-controller /home/mingh/robot-controller /home/mingh/robot-controller/build /home/mingh/robot-controller/build /home/mingh/robot-controller/build/CMakeFiles/RobotController.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RobotController.dir/depend

