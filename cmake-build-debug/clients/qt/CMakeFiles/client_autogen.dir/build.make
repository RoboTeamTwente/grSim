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
CMAKE_COMMAND = /home/visionpc/Downloads/clion-2018.1/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/visionpc/Downloads/clion-2018.1/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/visionpc/roboteamtwente/grSim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/visionpc/roboteamtwente/grSim/cmake-build-debug

# Utility rule file for client_autogen.

# Include the progress variables for this target.
include clients/qt/CMakeFiles/client_autogen.dir/progress.make

clients/qt/CMakeFiles/client_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/visionpc/roboteamtwente/grSim/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target client"
	cd /home/visionpc/roboteamtwente/grSim/cmake-build-debug/clients/qt && /home/visionpc/Downloads/clion-2018.1/bin/cmake/bin/cmake -E cmake_autogen /home/visionpc/roboteamtwente/grSim/cmake-build-debug/clients/qt/CMakeFiles/client_autogen.dir Debug

client_autogen: clients/qt/CMakeFiles/client_autogen
client_autogen: clients/qt/CMakeFiles/client_autogen.dir/build.make

.PHONY : client_autogen

# Rule to build all files generated by this target.
clients/qt/CMakeFiles/client_autogen.dir/build: client_autogen

.PHONY : clients/qt/CMakeFiles/client_autogen.dir/build

clients/qt/CMakeFiles/client_autogen.dir/clean:
	cd /home/visionpc/roboteamtwente/grSim/cmake-build-debug/clients/qt && $(CMAKE_COMMAND) -P CMakeFiles/client_autogen.dir/cmake_clean.cmake
.PHONY : clients/qt/CMakeFiles/client_autogen.dir/clean

clients/qt/CMakeFiles/client_autogen.dir/depend:
	cd /home/visionpc/roboteamtwente/grSim/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/visionpc/roboteamtwente/grSim /home/visionpc/roboteamtwente/grSim/clients/qt /home/visionpc/roboteamtwente/grSim/cmake-build-debug /home/visionpc/roboteamtwente/grSim/cmake-build-debug/clients/qt /home/visionpc/roboteamtwente/grSim/cmake-build-debug/clients/qt/CMakeFiles/client_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : clients/qt/CMakeFiles/client_autogen.dir/depend
