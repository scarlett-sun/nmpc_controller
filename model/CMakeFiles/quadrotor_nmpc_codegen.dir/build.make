# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model

# Include any dependencies generated for this target.
include CMakeFiles/quadrotor_nmpc_codegen.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quadrotor_nmpc_codegen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quadrotor_nmpc_codegen.dir/flags.make

CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.o: CMakeFiles/quadrotor_nmpc_codegen.dir/flags.make
CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.o: quadrotor_nmpc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.o -c /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model/quadrotor_nmpc.cpp

CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model/quadrotor_nmpc.cpp > CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.i

CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model/quadrotor_nmpc.cpp -o CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.s

# Object files for target quadrotor_nmpc_codegen
quadrotor_nmpc_codegen_OBJECTS = \
"CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.o"

# External object files for target quadrotor_nmpc_codegen
quadrotor_nmpc_codegen_EXTERNAL_OBJECTS =

quadrotor_nmpc_codegen: CMakeFiles/quadrotor_nmpc_codegen.dir/quadrotor_nmpc.cpp.o
quadrotor_nmpc_codegen: CMakeFiles/quadrotor_nmpc_codegen.dir/build.make
quadrotor_nmpc_codegen: /home/scarlett/Workspaces/ACADOtoolkit/build/lib/libacado_toolkit_s.so
quadrotor_nmpc_codegen: CMakeFiles/quadrotor_nmpc_codegen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable quadrotor_nmpc_codegen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadrotor_nmpc_codegen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quadrotor_nmpc_codegen.dir/build: quadrotor_nmpc_codegen

.PHONY : CMakeFiles/quadrotor_nmpc_codegen.dir/build

CMakeFiles/quadrotor_nmpc_codegen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quadrotor_nmpc_codegen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quadrotor_nmpc_codegen.dir/clean

CMakeFiles/quadrotor_nmpc_codegen.dir/depend:
	cd /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model /home/scarlett/Workspaces/rotors_integration_ws/src/perception_nmpc/nmpc_controller/model/CMakeFiles/quadrotor_nmpc_codegen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quadrotor_nmpc_codegen.dir/depend
