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
include octomap/CMakeFiles/octomap.dir/depend.make

# Include the progress variables for this target.
include octomap/CMakeFiles/octomap.dir/progress.make

# Include the compile flags for this target's objects.
include octomap/CMakeFiles/octomap.dir/flags.make

octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o: octomap/AbstractOcTree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/AbstractOcTree.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/AbstractOcTree.cpp

octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/AbstractOcTree.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/AbstractOcTree.cpp > CMakeFiles/octomap.dir/AbstractOcTree.cpp.i

octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/AbstractOcTree.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/AbstractOcTree.cpp -o CMakeFiles/octomap.dir/AbstractOcTree.cpp.s

octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.requires

octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.provides: octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.provides

octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o


octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o: octomap/AbstractOccupancyOcTree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/AbstractOccupancyOcTree.cpp

octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/AbstractOccupancyOcTree.cpp > CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.i

octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/AbstractOccupancyOcTree.cpp -o CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.s

octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.requires

octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.provides: octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.provides

octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o


octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o: octomap/Pointcloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/Pointcloud.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/Pointcloud.cpp

octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/Pointcloud.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/Pointcloud.cpp > CMakeFiles/octomap.dir/Pointcloud.cpp.i

octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/Pointcloud.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/Pointcloud.cpp -o CMakeFiles/octomap.dir/Pointcloud.cpp.s

octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.requires

octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.provides: octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.provides

octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o


octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o: octomap/ScanGraph.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/ScanGraph.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/ScanGraph.cpp

octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/ScanGraph.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/ScanGraph.cpp > CMakeFiles/octomap.dir/ScanGraph.cpp.i

octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/ScanGraph.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/ScanGraph.cpp -o CMakeFiles/octomap.dir/ScanGraph.cpp.s

octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.requires

octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.provides: octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.provides

octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o


octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o: octomap/CountingOcTree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/CountingOcTree.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/CountingOcTree.cpp

octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/CountingOcTree.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/CountingOcTree.cpp > CMakeFiles/octomap.dir/CountingOcTree.cpp.i

octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/CountingOcTree.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/CountingOcTree.cpp -o CMakeFiles/octomap.dir/CountingOcTree.cpp.s

octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.requires

octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.provides: octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.provides

octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o


octomap/CMakeFiles/octomap.dir/OcTree.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/OcTree.cpp.o: octomap/OcTree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object octomap/CMakeFiles/octomap.dir/OcTree.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/OcTree.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/OcTree.cpp

octomap/CMakeFiles/octomap.dir/OcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/OcTree.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/OcTree.cpp > CMakeFiles/octomap.dir/OcTree.cpp.i

octomap/CMakeFiles/octomap.dir/OcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/OcTree.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/OcTree.cpp -o CMakeFiles/octomap.dir/OcTree.cpp.s

octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.requires

octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.provides: octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.provides

octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/OcTree.cpp.o


octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o: octomap/OcTreeNode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/OcTreeNode.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/OcTreeNode.cpp

octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/OcTreeNode.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/OcTreeNode.cpp > CMakeFiles/octomap.dir/OcTreeNode.cpp.i

octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/OcTreeNode.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/OcTreeNode.cpp -o CMakeFiles/octomap.dir/OcTreeNode.cpp.s

octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.requires

octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.provides: octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.provides

octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o


octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o: octomap/OcTreeStamped.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/OcTreeStamped.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/OcTreeStamped.cpp

octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/OcTreeStamped.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/OcTreeStamped.cpp > CMakeFiles/octomap.dir/OcTreeStamped.cpp.i

octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/OcTreeStamped.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/OcTreeStamped.cpp -o CMakeFiles/octomap.dir/OcTreeStamped.cpp.s

octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.requires

octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.provides: octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.provides

octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o


octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o: octomap/OcTreeLUT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/OcTreeLUT.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/OcTreeLUT.cpp

octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/OcTreeLUT.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/OcTreeLUT.cpp > CMakeFiles/octomap.dir/OcTreeLUT.cpp.i

octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/OcTreeLUT.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/OcTreeLUT.cpp -o CMakeFiles/octomap.dir/OcTreeLUT.cpp.s

octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.requires

octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.provides: octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.provides

octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o


octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o: octomap/CMakeFiles/octomap.dir/flags.make
octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o: octomap/ColorOcTree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/octomap.dir/ColorOcTree.cpp.o -c /Users/jennings/Desktop/slam/project/octomap/ColorOcTree.cpp

octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap.dir/ColorOcTree.cpp.i"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jennings/Desktop/slam/project/octomap/ColorOcTree.cpp > CMakeFiles/octomap.dir/ColorOcTree.cpp.i

octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap.dir/ColorOcTree.cpp.s"
	cd /Users/jennings/Desktop/slam/project/octomap && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jennings/Desktop/slam/project/octomap/ColorOcTree.cpp -o CMakeFiles/octomap.dir/ColorOcTree.cpp.s

octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.requires:

.PHONY : octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.requires

octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.provides: octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.requires
	$(MAKE) -f octomap/CMakeFiles/octomap.dir/build.make octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.provides.build
.PHONY : octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.provides

octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.provides.build: octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o


# Object files for target octomap
octomap_OBJECTS = \
"CMakeFiles/octomap.dir/AbstractOcTree.cpp.o" \
"CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o" \
"CMakeFiles/octomap.dir/Pointcloud.cpp.o" \
"CMakeFiles/octomap.dir/ScanGraph.cpp.o" \
"CMakeFiles/octomap.dir/CountingOcTree.cpp.o" \
"CMakeFiles/octomap.dir/OcTree.cpp.o" \
"CMakeFiles/octomap.dir/OcTreeNode.cpp.o" \
"CMakeFiles/octomap.dir/OcTreeStamped.cpp.o" \
"CMakeFiles/octomap.dir/OcTreeLUT.cpp.o" \
"CMakeFiles/octomap.dir/ColorOcTree.cpp.o"

# External object files for target octomap
octomap_EXTERNAL_OBJECTS =

octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/OcTree.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/build.make
octomap/liboctomap.a: octomap/CMakeFiles/octomap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/jennings/Desktop/slam/project/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX static library liboctomap.a"
	cd /Users/jennings/Desktop/slam/project/octomap && $(CMAKE_COMMAND) -P CMakeFiles/octomap.dir/cmake_clean_target.cmake
	cd /Users/jennings/Desktop/slam/project/octomap && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/octomap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
octomap/CMakeFiles/octomap.dir/build: octomap/liboctomap.a

.PHONY : octomap/CMakeFiles/octomap.dir/build

octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/AbstractOcTree.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/AbstractOccupancyOcTree.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/Pointcloud.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/ScanGraph.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/CountingOcTree.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/OcTree.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/OcTreeNode.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/OcTreeStamped.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/OcTreeLUT.cpp.o.requires
octomap/CMakeFiles/octomap.dir/requires: octomap/CMakeFiles/octomap.dir/ColorOcTree.cpp.o.requires

.PHONY : octomap/CMakeFiles/octomap.dir/requires

octomap/CMakeFiles/octomap.dir/clean:
	cd /Users/jennings/Desktop/slam/project/octomap && $(CMAKE_COMMAND) -P CMakeFiles/octomap.dir/cmake_clean.cmake
.PHONY : octomap/CMakeFiles/octomap.dir/clean

octomap/CMakeFiles/octomap.dir/depend:
	cd /Users/jennings/Desktop/slam/project && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jennings/Desktop/slam/project /Users/jennings/Desktop/slam/project/octomap /Users/jennings/Desktop/slam/project /Users/jennings/Desktop/slam/project/octomap /Users/jennings/Desktop/slam/project/octomap/CMakeFiles/octomap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : octomap/CMakeFiles/octomap.dir/depend

