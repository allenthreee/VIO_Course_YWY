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
CMAKE_SOURCE_DIR = /home/allen/VIO_Course_草稿/ch5/hw_course5_new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build

# Include any dependencies generated for this target.
include frontend/CMakeFiles/slam_course_frontend.dir/depend.make

# Include the progress variables for this target.
include frontend/CMakeFiles/slam_course_frontend.dir/progress.make

# Include the compile flags for this target's objects.
include frontend/CMakeFiles/slam_course_frontend.dir/flags.make

frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o: frontend/CMakeFiles/slam_course_frontend.dir/flags.make
frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o: ../frontend/frame.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o"
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_frontend.dir/frame.cc.o -c /home/allen/VIO_Course_草稿/ch5/hw_course5_new/frontend/frame.cc

frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_frontend.dir/frame.cc.i"
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/allen/VIO_Course_草稿/ch5/hw_course5_new/frontend/frame.cc > CMakeFiles/slam_course_frontend.dir/frame.cc.i

frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_frontend.dir/frame.cc.s"
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/allen/VIO_Course_草稿/ch5/hw_course5_new/frontend/frame.cc -o CMakeFiles/slam_course_frontend.dir/frame.cc.s

frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.requires:

.PHONY : frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.requires

frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.provides: frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.requires
	$(MAKE) -f frontend/CMakeFiles/slam_course_frontend.dir/build.make frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.provides.build
.PHONY : frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.provides

frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.provides.build: frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o


# Object files for target slam_course_frontend
slam_course_frontend_OBJECTS = \
"CMakeFiles/slam_course_frontend.dir/frame.cc.o"

# External object files for target slam_course_frontend
slam_course_frontend_EXTERNAL_OBJECTS =

frontend/libslam_course_frontend.a: frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o
frontend/libslam_course_frontend.a: frontend/CMakeFiles/slam_course_frontend.dir/build.make
frontend/libslam_course_frontend.a: frontend/CMakeFiles/slam_course_frontend.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libslam_course_frontend.a"
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend && $(CMAKE_COMMAND) -P CMakeFiles/slam_course_frontend.dir/cmake_clean_target.cmake
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slam_course_frontend.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
frontend/CMakeFiles/slam_course_frontend.dir/build: frontend/libslam_course_frontend.a

.PHONY : frontend/CMakeFiles/slam_course_frontend.dir/build

frontend/CMakeFiles/slam_course_frontend.dir/requires: frontend/CMakeFiles/slam_course_frontend.dir/frame.cc.o.requires

.PHONY : frontend/CMakeFiles/slam_course_frontend.dir/requires

frontend/CMakeFiles/slam_course_frontend.dir/clean:
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend && $(CMAKE_COMMAND) -P CMakeFiles/slam_course_frontend.dir/cmake_clean.cmake
.PHONY : frontend/CMakeFiles/slam_course_frontend.dir/clean

frontend/CMakeFiles/slam_course_frontend.dir/depend:
	cd /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/allen/VIO_Course_草稿/ch5/hw_course5_new /home/allen/VIO_Course_草稿/ch5/hw_course5_new/frontend /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend /home/allen/VIO_Course_草稿/ch5/hw_course5_new/build/frontend/CMakeFiles/slam_course_frontend.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : frontend/CMakeFiles/slam_course_frontend.dir/depend

