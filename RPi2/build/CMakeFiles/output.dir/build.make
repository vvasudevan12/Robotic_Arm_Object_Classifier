# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/pi/Documents/ECE597/RBarm00

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Documents/ECE597/RBarm00/build

# Include any dependencies generated for this target.
include CMakeFiles/output.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/output.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/output.dir/flags.make

CMakeFiles/output.dir/src/rbarm04.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/rbarm04.cpp.o: ../src/rbarm04.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/ECE597/RBarm00/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/output.dir/src/rbarm04.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/output.dir/src/rbarm04.cpp.o -c /home/pi/Documents/ECE597/RBarm00/src/rbarm04.cpp

CMakeFiles/output.dir/src/rbarm04.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/rbarm04.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/ECE597/RBarm00/src/rbarm04.cpp > CMakeFiles/output.dir/src/rbarm04.cpp.i

CMakeFiles/output.dir/src/rbarm04.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/rbarm04.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/ECE597/RBarm00/src/rbarm04.cpp -o CMakeFiles/output.dir/src/rbarm04.cpp.s

CMakeFiles/output.dir/src/rbarm04.cpp.o.requires:

.PHONY : CMakeFiles/output.dir/src/rbarm04.cpp.o.requires

CMakeFiles/output.dir/src/rbarm04.cpp.o.provides: CMakeFiles/output.dir/src/rbarm04.cpp.o.requires
	$(MAKE) -f CMakeFiles/output.dir/build.make CMakeFiles/output.dir/src/rbarm04.cpp.o.provides.build
.PHONY : CMakeFiles/output.dir/src/rbarm04.cpp.o.provides

CMakeFiles/output.dir/src/rbarm04.cpp.o.provides.build: CMakeFiles/output.dir/src/rbarm04.cpp.o


CMakeFiles/output.dir/src/stdafx.cpp.o: CMakeFiles/output.dir/flags.make
CMakeFiles/output.dir/src/stdafx.cpp.o: ../src/stdafx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/ECE597/RBarm00/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/output.dir/src/stdafx.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/output.dir/src/stdafx.cpp.o -c /home/pi/Documents/ECE597/RBarm00/src/stdafx.cpp

CMakeFiles/output.dir/src/stdafx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/output.dir/src/stdafx.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/ECE597/RBarm00/src/stdafx.cpp > CMakeFiles/output.dir/src/stdafx.cpp.i

CMakeFiles/output.dir/src/stdafx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/output.dir/src/stdafx.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/ECE597/RBarm00/src/stdafx.cpp -o CMakeFiles/output.dir/src/stdafx.cpp.s

CMakeFiles/output.dir/src/stdafx.cpp.o.requires:

.PHONY : CMakeFiles/output.dir/src/stdafx.cpp.o.requires

CMakeFiles/output.dir/src/stdafx.cpp.o.provides: CMakeFiles/output.dir/src/stdafx.cpp.o.requires
	$(MAKE) -f CMakeFiles/output.dir/build.make CMakeFiles/output.dir/src/stdafx.cpp.o.provides.build
.PHONY : CMakeFiles/output.dir/src/stdafx.cpp.o.provides

CMakeFiles/output.dir/src/stdafx.cpp.o.provides.build: CMakeFiles/output.dir/src/stdafx.cpp.o


# Object files for target output
output_OBJECTS = \
"CMakeFiles/output.dir/src/rbarm04.cpp.o" \
"CMakeFiles/output.dir/src/stdafx.cpp.o"

# External object files for target output
output_EXTERNAL_OBJECTS =

output: CMakeFiles/output.dir/src/rbarm04.cpp.o
output: CMakeFiles/output.dir/src/stdafx.cpp.o
output: CMakeFiles/output.dir/build.make
output: CMakeFiles/output.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Documents/ECE597/RBarm00/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable output"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/output.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/output.dir/build: output

.PHONY : CMakeFiles/output.dir/build

CMakeFiles/output.dir/requires: CMakeFiles/output.dir/src/rbarm04.cpp.o.requires
CMakeFiles/output.dir/requires: CMakeFiles/output.dir/src/stdafx.cpp.o.requires

.PHONY : CMakeFiles/output.dir/requires

CMakeFiles/output.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/output.dir/cmake_clean.cmake
.PHONY : CMakeFiles/output.dir/clean

CMakeFiles/output.dir/depend:
	cd /home/pi/Documents/ECE597/RBarm00/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Documents/ECE597/RBarm00 /home/pi/Documents/ECE597/RBarm00 /home/pi/Documents/ECE597/RBarm00/build /home/pi/Documents/ECE597/RBarm00/build /home/pi/Documents/ECE597/RBarm00/build/CMakeFiles/output.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/output.dir/depend
