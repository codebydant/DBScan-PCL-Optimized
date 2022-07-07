# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /tmp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /tmp/build

# Include any dependencies generated for this target.
include dbscanlib/CMakeFiles/dbscan.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include dbscanlib/CMakeFiles/dbscan.dir/compiler_depend.make

# Include the progress variables for this target.
include dbscanlib/CMakeFiles/dbscan.dir/progress.make

# Include the compile flags for this target's objects.
include dbscanlib/CMakeFiles/dbscan.dir/flags.make

dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.o: dbscanlib/CMakeFiles/dbscan.dir/flags.make
dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.o: ../dbscanlib/src/cluster.cpp
dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.o: dbscanlib/CMakeFiles/dbscan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/tmp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.o"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.o -MF CMakeFiles/dbscan.dir/src/cluster.cpp.o.d -o CMakeFiles/dbscan.dir/src/cluster.cpp.o -c /tmp/dbscanlib/src/cluster.cpp

dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dbscan.dir/src/cluster.cpp.i"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /tmp/dbscanlib/src/cluster.cpp > CMakeFiles/dbscan.dir/src/cluster.cpp.i

dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dbscan.dir/src/cluster.cpp.s"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /tmp/dbscanlib/src/cluster.cpp -o CMakeFiles/dbscan.dir/src/cluster.cpp.s

dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.o: dbscanlib/CMakeFiles/dbscan.dir/flags.make
dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.o: ../dbscanlib/src/dbScan.cpp
dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.o: dbscanlib/CMakeFiles/dbscan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/tmp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.o"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.o -MF CMakeFiles/dbscan.dir/src/dbScan.cpp.o.d -o CMakeFiles/dbscan.dir/src/dbScan.cpp.o -c /tmp/dbscanlib/src/dbScan.cpp

dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dbscan.dir/src/dbScan.cpp.i"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /tmp/dbscanlib/src/dbScan.cpp > CMakeFiles/dbscan.dir/src/dbScan.cpp.i

dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dbscan.dir/src/dbScan.cpp.s"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /tmp/dbscanlib/src/dbScan.cpp -o CMakeFiles/dbscan.dir/src/dbScan.cpp.s

dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o: dbscanlib/CMakeFiles/dbscan.dir/flags.make
dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o: ../dbscanlib/src/OctreeGenerator.cpp
dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o: dbscanlib/CMakeFiles/dbscan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/tmp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o -MF CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o.d -o CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o -c /tmp/dbscanlib/src/OctreeGenerator.cpp

dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.i"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /tmp/dbscanlib/src/OctreeGenerator.cpp > CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.i

dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.s"
	cd /tmp/build/dbscanlib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /tmp/dbscanlib/src/OctreeGenerator.cpp -o CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.s

# Object files for target dbscan
dbscan_OBJECTS = \
"CMakeFiles/dbscan.dir/src/cluster.cpp.o" \
"CMakeFiles/dbscan.dir/src/dbScan.cpp.o" \
"CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o"

# External object files for target dbscan
dbscan_EXTERNAL_OBJECTS =

dbscanlib/libdbscan.so.1.0.3: dbscanlib/CMakeFiles/dbscan.dir/src/cluster.cpp.o
dbscanlib/libdbscan.so.1.0.3: dbscanlib/CMakeFiles/dbscan.dir/src/dbScan.cpp.o
dbscanlib/libdbscan.so.1.0.3: dbscanlib/CMakeFiles/dbscan.dir/src/OctreeGenerator.cpp.o
dbscanlib/libdbscan.so.1.0.3: dbscanlib/CMakeFiles/dbscan.dir/build.make
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libpcl_visualization.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libpcl_kdtree.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libflann_cpp.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libpcl_io.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libpcl_search.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libpcl_octree.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkChartsCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkInteractionImage.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkIOGeometry.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkjsoncpp.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkIOPLY.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingLOD.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkViewsContext2D.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkViewsCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkInteractionWidgets.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkFiltersModeling.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkInteractionStyle.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkFiltersExtraction.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkIOLegacy.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkIOCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingAnnotation.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingContext2D.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingFreeType.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkfreetype.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkImagingSources.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkIOImage.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkzlib.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkImagingCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingOpenGL2.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingUI.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkRenderingCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonColor.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkFiltersGeometry.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkFiltersSources.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkFiltersGeneral.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonComputationalGeometry.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkFiltersCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonExecutionModel.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonDataModel.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonMisc.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonTransforms.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonMath.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkkissfft.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkglew.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libGL.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libGLU.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libX11.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtkCommonCore.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libvtksys.so.9.1.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libexecinfo.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libexecinfo.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libpcl_common.so
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libboost_system.so.1.77.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libboost_filesystem.so.1.77.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libboost_atomic.so.1.77.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libboost_date_time.so.1.77.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libboost_iostreams.so.1.77.0
dbscanlib/libdbscan.so.1.0.3: /usr/lib/libboost_serialization.so.1.77.0
dbscanlib/libdbscan.so.1.0.3: dbscanlib/CMakeFiles/dbscan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/tmp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libdbscan.so"
	cd /tmp/build/dbscanlib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dbscan.dir/link.txt --verbose=$(VERBOSE)
	cd /tmp/build/dbscanlib && $(CMAKE_COMMAND) -E cmake_symlink_library libdbscan.so.1.0.3 libdbscan.so.1 libdbscan.so

dbscanlib/libdbscan.so.1: dbscanlib/libdbscan.so.1.0.3
	@$(CMAKE_COMMAND) -E touch_nocreate dbscanlib/libdbscan.so.1

dbscanlib/libdbscan.so: dbscanlib/libdbscan.so.1.0.3
	@$(CMAKE_COMMAND) -E touch_nocreate dbscanlib/libdbscan.so

# Rule to build all files generated by this target.
dbscanlib/CMakeFiles/dbscan.dir/build: dbscanlib/libdbscan.so
.PHONY : dbscanlib/CMakeFiles/dbscan.dir/build

dbscanlib/CMakeFiles/dbscan.dir/clean:
	cd /tmp/build/dbscanlib && $(CMAKE_COMMAND) -P CMakeFiles/dbscan.dir/cmake_clean.cmake
.PHONY : dbscanlib/CMakeFiles/dbscan.dir/clean

dbscanlib/CMakeFiles/dbscan.dir/depend:
	cd /tmp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /tmp /tmp/dbscanlib /tmp/build /tmp/build/dbscanlib /tmp/build/dbscanlib/CMakeFiles/dbscan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dbscanlib/CMakeFiles/dbscan.dir/depend
