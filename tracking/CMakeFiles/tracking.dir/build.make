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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.4.3/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.4.3/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jennings/Desktop/slam/project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jennings/Desktop/slam/project

# Include any dependencies generated for this target.
include tracking/CMakeFiles/tracking.dir/depend.make

# Include the progress variables for this target.
include tracking/CMakeFiles/tracking.dir/progress.make

# Include the compile flags for this target's objects.
include tracking/CMakeFiles/tracking.dir/flags.make

tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o: tracking/CMakeFiles/tracking.dir/flags.make
tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o: tracking/assignmentoptimal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/assignmentoptimal.cpp.o -c /Users/jennings/Desktop/slam/project/tracking/assignmentoptimal.cpp

tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/assignmentoptimal.cpp.i"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/tracking/assignmentoptimal.cpp > CMakeFiles/tracking.dir/assignmentoptimal.cpp.i

tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/assignmentoptimal.cpp.s"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/tracking/assignmentoptimal.cpp -o CMakeFiles/tracking.dir/assignmentoptimal.cpp.s

tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.requires

tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.provides: tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking.dir/build.make tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.provides

tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.provides.build: tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o


tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o: tracking/CMakeFiles/tracking.dir/flags.make
tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o: tracking/TrackManager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/TrackManager.cpp.o -c /Users/jennings/Desktop/slam/project/tracking/TrackManager.cpp

tracking/CMakeFiles/tracking.dir/TrackManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/TrackManager.cpp.i"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/tracking/TrackManager.cpp > CMakeFiles/tracking.dir/TrackManager.cpp.i

tracking/CMakeFiles/tracking.dir/TrackManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/TrackManager.cpp.s"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/tracking/TrackManager.cpp -o CMakeFiles/tracking.dir/TrackManager.cpp.s

tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.requires

tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.provides: tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking.dir/build.make tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.provides

tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.provides.build: tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o


tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o: tracking/CMakeFiles/tracking.dir/flags.make
tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o: tracking/TrackTarget.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/TrackTarget.cpp.o -c /Users/jennings/Desktop/slam/project/tracking/TrackTarget.cpp

tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/TrackTarget.cpp.i"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/tracking/TrackTarget.cpp > CMakeFiles/tracking.dir/TrackTarget.cpp.i

tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/TrackTarget.cpp.s"
	cd /Users/jennings/Desktop/slam/project/tracking && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/tracking/TrackTarget.cpp -o CMakeFiles/tracking.dir/TrackTarget.cpp.s

tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.requires:

.PHONY : tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.requires

tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.provides: tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.requires
	$(MAKE) -f tracking/CMakeFiles/tracking.dir/build.make tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.provides.build
.PHONY : tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.provides

tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.provides.build: tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o


# Object files for target tracking
tracking_OBJECTS = \
"CMakeFiles/tracking.dir/assignmentoptimal.cpp.o" \
"CMakeFiles/tracking.dir/TrackManager.cpp.o" \
"CMakeFiles/tracking.dir/TrackTarget.cpp.o"

# External object files for target tracking
tracking_EXTERNAL_OBJECTS =

tracking/libtracking.a: tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o
tracking/libtracking.a: tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o
tracking/libtracking.a: tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o
tracking/libtracking.a: tracking/CMakeFiles/tracking.dir/build.make
tracking/libtracking.a: tracking/CMakeFiles/tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libtracking.a"
	cd /Users/jennings/Desktop/slam/project/tracking && $(CMAKE_COMMAND) -P CMakeFiles/tracking.dir/cmake_clean_target.cmake
	cd /Users/jennings/Desktop/slam/project/tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tracking/CMakeFiles/tracking.dir/build: tracking/libtracking.a

.PHONY : tracking/CMakeFiles/tracking.dir/build

tracking/CMakeFiles/tracking.dir/requires: tracking/CMakeFiles/tracking.dir/assignmentoptimal.cpp.o.requires
tracking/CMakeFiles/tracking.dir/requires: tracking/CMakeFiles/tracking.dir/TrackManager.cpp.o.requires
tracking/CMakeFiles/tracking.dir/requires: tracking/CMakeFiles/tracking.dir/TrackTarget.cpp.o.requires

.PHONY : tracking/CMakeFiles/tracking.dir/requires

tracking/CMakeFiles/tracking.dir/clean:
	cd /Users/jennings/Desktop/slam/project/tracking && $(CMAKE_COMMAND) -P CMakeFiles/tracking.dir/cmake_clean.cmake
.PHONY : tracking/CMakeFiles/tracking.dir/clean

tracking/CMakeFiles/tracking.dir/depend:
	cd /Users/jennings/Desktop/slam/project && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jennings/Desktop/slam/project /Users/jennings/Desktop/slam/project/tracking /Users/jennings/Desktop/slam/project /Users/jennings/Desktop/slam/project/tracking /Users/jennings/Desktop/slam/project/tracking/CMakeFiles/tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tracking/CMakeFiles/tracking.dir/depend
