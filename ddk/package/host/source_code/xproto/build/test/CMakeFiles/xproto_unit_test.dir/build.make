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
CMAKE_SOURCE_DIR = /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build

# Include any dependencies generated for this target.
include test/CMakeFiles/xproto_unit_test.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/xproto_unit_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/xproto_unit_test.dir/flags.make

test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o: test/CMakeFiles/xproto_unit_test.dir/flags.make
test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o: ../test/gtest_main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/gtest_main.cc

test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xproto_unit_test.dir/gtest_main.cc.i"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/gtest_main.cc > CMakeFiles/xproto_unit_test.dir/gtest_main.cc.i

test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xproto_unit_test.dir/gtest_main.cc.s"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/gtest_main.cc -o CMakeFiles/xproto_unit_test.dir/gtest_main.cc.s

test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.requires:

.PHONY : test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.requires

test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.provides: test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.requires
	$(MAKE) -f test/CMakeFiles/xproto_unit_test.dir/build.make test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.provides.build
.PHONY : test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.provides

test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.provides.build: test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o


test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o: test/CMakeFiles/xproto_unit_test.dir/flags.make
test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o: ../test/test_api.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xproto_unit_test.dir/test_api.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/test_api.cc

test/CMakeFiles/xproto_unit_test.dir/test_api.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xproto_unit_test.dir/test_api.cc.i"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/test_api.cc > CMakeFiles/xproto_unit_test.dir/test_api.cc.i

test/CMakeFiles/xproto_unit_test.dir/test_api.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xproto_unit_test.dir/test_api.cc.s"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/test_api.cc -o CMakeFiles/xproto_unit_test.dir/test_api.cc.s

test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.requires:

.PHONY : test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.requires

test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.provides: test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.requires
	$(MAKE) -f test/CMakeFiles/xproto_unit_test.dir/build.make test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.provides.build
.PHONY : test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.provides

test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.provides.build: test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o


test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o: test/CMakeFiles/xproto_unit_test.dir/flags.make
test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o: ../test/test_xplugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/test_xplugin.cc

test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.i"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/test_xplugin.cc > CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.i

test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.s"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test/test_xplugin.cc -o CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.s

test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.requires:

.PHONY : test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.requires

test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.provides: test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.requires
	$(MAKE) -f test/CMakeFiles/xproto_unit_test.dir/build.make test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.provides.build
.PHONY : test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.provides

test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.provides.build: test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o


# Object files for target xproto_unit_test
xproto_unit_test_OBJECTS = \
"CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o" \
"CMakeFiles/xproto_unit_test.dir/test_api.cc.o" \
"CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o"

# External object files for target xproto_unit_test
xproto_unit_test_EXTERNAL_OBJECTS =

test/xproto_unit_test: test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o
test/xproto_unit_test: test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o
test/xproto_unit_test: test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o
test/xproto_unit_test: test/CMakeFiles/xproto_unit_test.dir/build.make
test/xproto_unit_test: libxproto.so
test/xproto_unit_test: test/CMakeFiles/xproto_unit_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable xproto_unit_test"
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xproto_unit_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/xproto_unit_test.dir/build: test/xproto_unit_test

.PHONY : test/CMakeFiles/xproto_unit_test.dir/build

test/CMakeFiles/xproto_unit_test.dir/requires: test/CMakeFiles/xproto_unit_test.dir/gtest_main.cc.o.requires
test/CMakeFiles/xproto_unit_test.dir/requires: test/CMakeFiles/xproto_unit_test.dir/test_api.cc.o.requires
test/CMakeFiles/xproto_unit_test.dir/requires: test/CMakeFiles/xproto_unit_test.dir/test_xplugin.cc.o.requires

.PHONY : test/CMakeFiles/xproto_unit_test.dir/requires

test/CMakeFiles/xproto_unit_test.dir/clean:
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test && $(CMAKE_COMMAND) -P CMakeFiles/xproto_unit_test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/xproto_unit_test.dir/clean

test/CMakeFiles/xproto_unit_test.dir/depend:
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/test /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test/CMakeFiles/xproto_unit_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/xproto_unit_test.dir/depend

