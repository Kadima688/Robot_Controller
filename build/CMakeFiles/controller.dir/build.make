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
include CMakeFiles/controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/controller.dir/flags.make

CMakeFiles/controller.dir/RobotController.cpp.o: CMakeFiles/controller.dir/flags.make
CMakeFiles/controller.dir/RobotController.cpp.o: ../RobotController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mingh/robot-controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/controller.dir/RobotController.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller.dir/RobotController.cpp.o -c /home/mingh/robot-controller/RobotController.cpp

CMakeFiles/controller.dir/RobotController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/RobotController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mingh/robot-controller/RobotController.cpp > CMakeFiles/controller.dir/RobotController.cpp.i

CMakeFiles/controller.dir/RobotController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/RobotController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mingh/robot-controller/RobotController.cpp -o CMakeFiles/controller.dir/RobotController.cpp.s

CMakeFiles/controller.dir/RobotController.cpp.o.requires:

.PHONY : CMakeFiles/controller.dir/RobotController.cpp.o.requires

CMakeFiles/controller.dir/RobotController.cpp.o.provides: CMakeFiles/controller.dir/RobotController.cpp.o.requires
	$(MAKE) -f CMakeFiles/controller.dir/build.make CMakeFiles/controller.dir/RobotController.cpp.o.provides.build
.PHONY : CMakeFiles/controller.dir/RobotController.cpp.o.provides

CMakeFiles/controller.dir/RobotController.cpp.o.provides.build: CMakeFiles/controller.dir/RobotController.cpp.o


CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o: CMakeFiles/controller.dir/flags.make
CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o: ../Module/robot/src/MhDh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mingh/robot-controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o -c /home/mingh/robot-controller/Module/robot/src/MhDh.cpp

CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mingh/robot-controller/Module/robot/src/MhDh.cpp > CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.i

CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mingh/robot-controller/Module/robot/src/MhDh.cpp -o CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.s

CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.requires:

.PHONY : CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.requires

CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.provides: CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.requires
	$(MAKE) -f CMakeFiles/controller.dir/build.make CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.provides.build
.PHONY : CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.provides

CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.provides.build: CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o


# Object files for target controller
controller_OBJECTS = \
"CMakeFiles/controller.dir/RobotController.cpp.o" \
"CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o"

# External object files for target controller
controller_EXTERNAL_OBJECTS =

controller: CMakeFiles/controller.dir/RobotController.cpp.o
controller: CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o
controller: CMakeFiles/controller.dir/build.make
controller: CMakeFiles/controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mingh/robot-controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/controller.dir/build: controller

.PHONY : CMakeFiles/controller.dir/build

CMakeFiles/controller.dir/requires: CMakeFiles/controller.dir/RobotController.cpp.o.requires
CMakeFiles/controller.dir/requires: CMakeFiles/controller.dir/Module/robot/src/MhDh.cpp.o.requires

.PHONY : CMakeFiles/controller.dir/requires

CMakeFiles/controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/controller.dir/clean

CMakeFiles/controller.dir/depend:
	cd /home/mingh/robot-controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mingh/robot-controller /home/mingh/robot-controller /home/mingh/robot-controller/build /home/mingh/robot-controller/build /home/mingh/robot-controller/build/CMakeFiles/controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/controller.dir/depend

