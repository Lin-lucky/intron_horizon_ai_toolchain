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
CMAKE_SOURCE_DIR = /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build

# Include any dependencies generated for this target.
include example/CMakeFiles/rtsp_server_example.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/rtsp_server_example.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/rtsp_server_example.dir/flags.make

example/CMakeFiles/rtsp_server_example.dir/main.cc.o: example/CMakeFiles/rtsp_server_example.dir/flags.make
example/CMakeFiles/rtsp_server_example.dir/main.cc.o: ../example/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/CMakeFiles/rtsp_server_example.dir/main.cc.o"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtsp_server_example.dir/main.cc.o -c /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/main.cc

example/CMakeFiles/rtsp_server_example.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtsp_server_example.dir/main.cc.i"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/main.cc > CMakeFiles/rtsp_server_example.dir/main.cc.i

example/CMakeFiles/rtsp_server_example.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtsp_server_example.dir/main.cc.s"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/main.cc -o CMakeFiles/rtsp_server_example.dir/main.cc.s

example/CMakeFiles/rtsp_server_example.dir/main.cc.o.requires:

.PHONY : example/CMakeFiles/rtsp_server_example.dir/main.cc.o.requires

example/CMakeFiles/rtsp_server_example.dir/main.cc.o.provides: example/CMakeFiles/rtsp_server_example.dir/main.cc.o.requires
	$(MAKE) -f example/CMakeFiles/rtsp_server_example.dir/build.make example/CMakeFiles/rtsp_server_example.dir/main.cc.o.provides.build
.PHONY : example/CMakeFiles/rtsp_server_example.dir/main.cc.o.provides

example/CMakeFiles/rtsp_server_example.dir/main.cc.o.provides.build: example/CMakeFiles/rtsp_server_example.dir/main.cc.o


example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o: example/CMakeFiles/rtsp_server_example.dir/flags.make
example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o: ../example/src/media_producer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o -c /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/src/media_producer.cc

example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.i"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/src/media_producer.cc > CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.i

example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.s"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/src/media_producer.cc -o CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.s

example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.requires:

.PHONY : example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.requires

example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.provides: example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.requires
	$(MAKE) -f example/CMakeFiles/rtsp_server_example.dir/build.make example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.provides.build
.PHONY : example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.provides

example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.provides.build: example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o


# Object files for target rtsp_server_example
rtsp_server_example_OBJECTS = \
"CMakeFiles/rtsp_server_example.dir/main.cc.o" \
"CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o"

# External object files for target rtsp_server_example
rtsp_server_example_EXTERNAL_OBJECTS =

example/rtsp_server_example: example/CMakeFiles/rtsp_server_example.dir/main.cc.o
example/rtsp_server_example: example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o
example/rtsp_server_example: example/CMakeFiles/rtsp_server_example.dir/build.make
example/rtsp_server_example: librtsp_server.so
example/rtsp_server_example: example/CMakeFiles/rtsp_server_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable rtsp_server_example"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rtsp_server_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/rtsp_server_example.dir/build: example/rtsp_server_example

.PHONY : example/CMakeFiles/rtsp_server_example.dir/build

example/CMakeFiles/rtsp_server_example.dir/requires: example/CMakeFiles/rtsp_server_example.dir/main.cc.o.requires
example/CMakeFiles/rtsp_server_example.dir/requires: example/CMakeFiles/rtsp_server_example.dir/src/media_producer.cc.o.requires

.PHONY : example/CMakeFiles/rtsp_server_example.dir/requires

example/CMakeFiles/rtsp_server_example.dir/clean:
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example && $(CMAKE_COMMAND) -P CMakeFiles/rtsp_server_example.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/rtsp_server_example.dir/clean

example/CMakeFiles/rtsp_server_example.dir/depend:
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example/CMakeFiles/rtsp_server_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/rtsp_server_example.dir/depend

