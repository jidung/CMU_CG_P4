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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build

# Include any dependencies generated for this target.
include glew/CMakeFiles/glew.dir/depend.make

# Include the progress variables for this target.
include glew/CMakeFiles/glew.dir/progress.make

# Include the compile flags for this target's objects.
include glew/CMakeFiles/glew.dir/flags.make

glew/CMakeFiles/glew.dir/glew.cpp.o: glew/CMakeFiles/glew.dir/flags.make
glew/CMakeFiles/glew.dir/glew.cpp.o: /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/glew/glew.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object glew/CMakeFiles/glew.dir/glew.cpp.o"
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/glew.dir/glew.cpp.o -c /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/glew/glew.cpp

glew/CMakeFiles/glew.dir/glew.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/glew.dir/glew.cpp.i"
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/glew/glew.cpp > CMakeFiles/glew.dir/glew.cpp.i

glew/CMakeFiles/glew.dir/glew.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/glew.dir/glew.cpp.s"
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/glew/glew.cpp -o CMakeFiles/glew.dir/glew.cpp.s

glew/CMakeFiles/glew.dir/glew.cpp.o.requires:
.PHONY : glew/CMakeFiles/glew.dir/glew.cpp.o.requires

glew/CMakeFiles/glew.dir/glew.cpp.o.provides: glew/CMakeFiles/glew.dir/glew.cpp.o.requires
	$(MAKE) -f glew/CMakeFiles/glew.dir/build.make glew/CMakeFiles/glew.dir/glew.cpp.o.provides.build
.PHONY : glew/CMakeFiles/glew.dir/glew.cpp.o.provides

glew/CMakeFiles/glew.dir/glew.cpp.o.provides.build: glew/CMakeFiles/glew.dir/glew.cpp.o

# Object files for target glew
glew_OBJECTS = \
"CMakeFiles/glew.dir/glew.cpp.o"

# External object files for target glew
glew_EXTERNAL_OBJECTS =

glew/libglew.a: glew/CMakeFiles/glew.dir/glew.cpp.o
glew/libglew.a: glew/CMakeFiles/glew.dir/build.make
glew/libglew.a: glew/CMakeFiles/glew.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libglew.a"
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean_target.cmake
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glew.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
glew/CMakeFiles/glew.dir/build: glew/libglew.a
.PHONY : glew/CMakeFiles/glew.dir/build

glew/CMakeFiles/glew.dir/requires: glew/CMakeFiles/glew.dir/glew.cpp.o.requires
.PHONY : glew/CMakeFiles/glew.dir/requires

glew/CMakeFiles/glew.dir/clean:
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean.cmake
.PHONY : glew/CMakeFiles/glew.dir/clean

glew/CMakeFiles/glew.dir/depend:
	cd /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/glew /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/glew/CMakeFiles/glew.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : glew/CMakeFiles/glew.dir/depend

