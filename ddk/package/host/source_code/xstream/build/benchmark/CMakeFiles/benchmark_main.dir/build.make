# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build

# Include any dependencies generated for this target.
include benchmark/CMakeFiles/benchmark_main.dir/depend.make

# Include the progress variables for this target.
include benchmark/CMakeFiles/benchmark_main.dir/progress.make

# Include the compile flags for this target's objects.
include benchmark/CMakeFiles/benchmark_main.dir/flags.make

benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o: benchmark/CMakeFiles/benchmark_main.dir/flags.make
benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o: ../benchmark/benchmark_main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/benchmark_main.dir/benchmark_main.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark/benchmark_main.cc

benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/benchmark_main.dir/benchmark_main.cc.i"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark/benchmark_main.cc > CMakeFiles/benchmark_main.dir/benchmark_main.cc.i

benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/benchmark_main.dir/benchmark_main.cc.s"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark/benchmark_main.cc -o CMakeFiles/benchmark_main.dir/benchmark_main.cc.s

benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.requires:

.PHONY : benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.requires

benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.provides: benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.requires
	$(MAKE) -f benchmark/CMakeFiles/benchmark_main.dir/build.make benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.provides.build
.PHONY : benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.provides

benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.provides.build: benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o


benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o: benchmark/CMakeFiles/benchmark_main.dir/flags.make
benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o: ../benchmark/method/method_factory.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/benchmark_main.dir/method/method_factory.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark/method/method_factory.cc

benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/benchmark_main.dir/method/method_factory.cc.i"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark/method/method_factory.cc > CMakeFiles/benchmark_main.dir/method/method_factory.cc.i

benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/benchmark_main.dir/method/method_factory.cc.s"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark/method/method_factory.cc -o CMakeFiles/benchmark_main.dir/method/method_factory.cc.s

benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.requires:

.PHONY : benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.requires

benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.provides: benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.requires
	$(MAKE) -f benchmark/CMakeFiles/benchmark_main.dir/build.make benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.provides.build
.PHONY : benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.provides

benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.provides.build: benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o


# Object files for target benchmark_main
benchmark_main_OBJECTS = \
"CMakeFiles/benchmark_main.dir/benchmark_main.cc.o" \
"CMakeFiles/benchmark_main.dir/method/method_factory.cc.o"

# External object files for target benchmark_main
benchmark_main_EXTERNAL_OBJECTS =

benchmark/benchmark_main: benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o
benchmark/benchmark_main: benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o
benchmark/benchmark_main: benchmark/CMakeFiles/benchmark_main.dir/build.make
benchmark/benchmark_main: libxstream.so
benchmark/benchmark_main: benchmark/CMakeFiles/benchmark_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable benchmark_main"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/benchmark_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
benchmark/CMakeFiles/benchmark_main.dir/build: benchmark/benchmark_main

.PHONY : benchmark/CMakeFiles/benchmark_main.dir/build

benchmark/CMakeFiles/benchmark_main.dir/requires: benchmark/CMakeFiles/benchmark_main.dir/benchmark_main.cc.o.requires
benchmark/CMakeFiles/benchmark_main.dir/requires: benchmark/CMakeFiles/benchmark_main.dir/method/method_factory.cc.o.requires

.PHONY : benchmark/CMakeFiles/benchmark_main.dir/requires

benchmark/CMakeFiles/benchmark_main.dir/clean:
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark && $(CMAKE_COMMAND) -P CMakeFiles/benchmark_main.dir/cmake_clean.cmake
.PHONY : benchmark/CMakeFiles/benchmark_main.dir/clean

benchmark/CMakeFiles/benchmark_main.dir/depend:
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/benchmark /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark/CMakeFiles/benchmark_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : benchmark/CMakeFiles/benchmark_main.dir/depend
