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
include test/CMakeFiles/rtsp_server_gtest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/rtsp_server_gtest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/rtsp_server_gtest.dir/flags.make

test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o: test/CMakeFiles/rtsp_server_gtest.dir/flags.make
test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o: ../test/gtest_main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o -c /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test/gtest_main.cc

test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.i"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test/gtest_main.cc > CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.i

test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.s"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test/gtest_main.cc -o CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.s

test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.requires:

.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.requires

test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.provides: test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.requires
	$(MAKE) -f test/CMakeFiles/rtsp_server_gtest.dir/build.make test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.provides.build
.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.provides

test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.provides.build: test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o


test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o: test/CMakeFiles/rtsp_server_gtest.dir/flags.make
test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o: ../test/rtsp_server_gtest.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o -c /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test/rtsp_server_gtest.cc

test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.i"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test/rtsp_server_gtest.cc > CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.i

test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.s"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test/rtsp_server_gtest.cc -o CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.s

test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.requires:

.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.requires

test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.provides: test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.requires
	$(MAKE) -f test/CMakeFiles/rtsp_server_gtest.dir/build.make test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.provides.build
.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.provides

test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.provides.build: test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o


# Object files for target rtsp_server_gtest
rtsp_server_gtest_OBJECTS = \
"CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o" \
"CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o"

# External object files for target rtsp_server_gtest
rtsp_server_gtest_EXTERNAL_OBJECTS =

test/rtsp_server_gtest: test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o
test/rtsp_server_gtest: test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o
test/rtsp_server_gtest: test/CMakeFiles/rtsp_server_gtest.dir/build.make
test/rtsp_server_gtest: librtsp_server.so
test/rtsp_server_gtest: test/CMakeFiles/rtsp_server_gtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable rtsp_server_gtest"
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rtsp_server_gtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/rtsp_server_gtest.dir/build: test/rtsp_server_gtest

.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/build

test/CMakeFiles/rtsp_server_gtest.dir/requires: test/CMakeFiles/rtsp_server_gtest.dir/gtest_main.cc.o.requires
test/CMakeFiles/rtsp_server_gtest.dir/requires: test/CMakeFiles/rtsp_server_gtest.dir/rtsp_server_gtest.cc.o.requires

.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/requires

test/CMakeFiles/rtsp_server_gtest.dir/clean:
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test && $(CMAKE_COMMAND) -P CMakeFiles/rtsp_server_gtest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/clean

test/CMakeFiles/rtsp_server_gtest.dir/depend:
	cd /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/test /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/test/CMakeFiles/rtsp_server_gtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/rtsp_server_gtest.dir/depend

