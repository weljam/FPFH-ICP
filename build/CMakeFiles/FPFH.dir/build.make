# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /home/cwj/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/cwj/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cwj/PointCloud

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cwj/PointCloud/build

# Include any dependencies generated for this target.
include CMakeFiles/FPFH.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/FPFH.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FPFH.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FPFH.dir/flags.make

CMakeFiles/FPFH.dir/src/FPFH.cpp.o: CMakeFiles/FPFH.dir/flags.make
CMakeFiles/FPFH.dir/src/FPFH.cpp.o: /home/cwj/PointCloud/src/FPFH.cpp
CMakeFiles/FPFH.dir/src/FPFH.cpp.o: CMakeFiles/FPFH.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/cwj/PointCloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FPFH.dir/src/FPFH.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/FPFH.dir/src/FPFH.cpp.o -MF CMakeFiles/FPFH.dir/src/FPFH.cpp.o.d -o CMakeFiles/FPFH.dir/src/FPFH.cpp.o -c /home/cwj/PointCloud/src/FPFH.cpp

CMakeFiles/FPFH.dir/src/FPFH.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/FPFH.dir/src/FPFH.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cwj/PointCloud/src/FPFH.cpp > CMakeFiles/FPFH.dir/src/FPFH.cpp.i

CMakeFiles/FPFH.dir/src/FPFH.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/FPFH.dir/src/FPFH.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cwj/PointCloud/src/FPFH.cpp -o CMakeFiles/FPFH.dir/src/FPFH.cpp.s

# Object files for target FPFH
FPFH_OBJECTS = \
"CMakeFiles/FPFH.dir/src/FPFH.cpp.o"

# External object files for target FPFH
FPFH_EXTERNAL_OBJECTS =

/home/cwj/PointCloud/bin/FPFH: CMakeFiles/FPFH.dir/src/FPFH.cpp.o
/home/cwj/PointCloud/bin/FPFH: CMakeFiles/FPFH.dir/build.make
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_people.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/libOpenNI.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/libOpenNI2.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libfreetype.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libz.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libjpeg.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpng.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libtiff.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libexpat.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_features.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_search.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_io.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libpcl_common.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libfreetype.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libz.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libGLEW.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libSM.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libICE.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libX11.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libXext.so
/home/cwj/PointCloud/bin/FPFH: /usr/lib/x86_64-linux-gnu/libXt.so
/home/cwj/PointCloud/bin/FPFH: CMakeFiles/FPFH.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/cwj/PointCloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/cwj/PointCloud/bin/FPFH"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FPFH.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FPFH.dir/build: /home/cwj/PointCloud/bin/FPFH
.PHONY : CMakeFiles/FPFH.dir/build

CMakeFiles/FPFH.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FPFH.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FPFH.dir/clean

CMakeFiles/FPFH.dir/depend:
	cd /home/cwj/PointCloud/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cwj/PointCloud /home/cwj/PointCloud /home/cwj/PointCloud/build /home/cwj/PointCloud/build /home/cwj/PointCloud/build/CMakeFiles/FPFH.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/FPFH.dir/depend

