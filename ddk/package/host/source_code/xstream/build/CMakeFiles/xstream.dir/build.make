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
include CMakeFiles/xstream.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xstream.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xstream.dir/flags.make

CMakeFiles/xstream.dir/src/common/com_func.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/common/com_func.cc.o: ../src/common/com_func.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xstream.dir/src/common/com_func.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/common/com_func.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/common/com_func.cc

CMakeFiles/xstream.dir/src/common/com_func.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/common/com_func.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/common/com_func.cc > CMakeFiles/xstream.dir/src/common/com_func.cc.i

CMakeFiles/xstream.dir/src/common/com_func.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/common/com_func.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/common/com_func.cc -o CMakeFiles/xstream.dir/src/common/com_func.cc.s

CMakeFiles/xstream.dir/src/common/com_func.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/common/com_func.cc.o.requires

CMakeFiles/xstream.dir/src/common/com_func.cc.o.provides: CMakeFiles/xstream.dir/src/common/com_func.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/common/com_func.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/common/com_func.cc.o.provides

CMakeFiles/xstream.dir/src/common/com_func.cc.o.provides.build: CMakeFiles/xstream.dir/src/common/com_func.cc.o


CMakeFiles/xstream.dir/src/profiler.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/profiler.cc.o: ../src/profiler.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/xstream.dir/src/profiler.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/profiler.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/profiler.cc

CMakeFiles/xstream.dir/src/profiler.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/profiler.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/profiler.cc > CMakeFiles/xstream.dir/src/profiler.cc.i

CMakeFiles/xstream.dir/src/profiler.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/profiler.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/profiler.cc -o CMakeFiles/xstream.dir/src/profiler.cc.s

CMakeFiles/xstream.dir/src/profiler.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/profiler.cc.o.requires

CMakeFiles/xstream.dir/src/profiler.cc.o.provides: CMakeFiles/xstream.dir/src/profiler.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/profiler.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/profiler.cc.o.provides

CMakeFiles/xstream.dir/src/profiler.cc.o.provides.build: CMakeFiles/xstream.dir/src/profiler.cc.o


CMakeFiles/xstream.dir/src/xstream.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/xstream.cc.o: ../src/xstream.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/xstream.dir/src/xstream.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/xstream.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xstream.cc

CMakeFiles/xstream.dir/src/xstream.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/xstream.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xstream.cc > CMakeFiles/xstream.dir/src/xstream.cc.i

CMakeFiles/xstream.dir/src/xstream.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/xstream.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xstream.cc -o CMakeFiles/xstream.dir/src/xstream.cc.s

CMakeFiles/xstream.dir/src/xstream.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/xstream.cc.o.requires

CMakeFiles/xstream.dir/src/xstream.cc.o.provides: CMakeFiles/xstream.dir/src/xstream.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/xstream.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/xstream.cc.o.provides

CMakeFiles/xstream.dir/src/xstream.cc.o.provides.build: CMakeFiles/xstream.dir/src/xstream.cc.o


CMakeFiles/xstream.dir/src/xstream_config.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/xstream_config.cc.o: ../src/xstream_config.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/xstream.dir/src/xstream_config.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/xstream_config.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xstream_config.cc

CMakeFiles/xstream.dir/src/xstream_config.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/xstream_config.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xstream_config.cc > CMakeFiles/xstream.dir/src/xstream_config.cc.i

CMakeFiles/xstream.dir/src/xstream_config.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/xstream_config.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xstream_config.cc -o CMakeFiles/xstream.dir/src/xstream_config.cc.s

CMakeFiles/xstream.dir/src/xstream_config.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/xstream_config.cc.o.requires

CMakeFiles/xstream.dir/src/xstream_config.cc.o.provides: CMakeFiles/xstream.dir/src/xstream_config.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/xstream_config.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/xstream_config.cc.o.provides

CMakeFiles/xstream.dir/src/xstream_config.cc.o.provides.build: CMakeFiles/xstream.dir/src/xstream_config.cc.o


CMakeFiles/xstream.dir/src/method.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/method.cc.o: ../src/method.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/xstream.dir/src/method.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/method.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/method.cc

CMakeFiles/xstream.dir/src/method.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/method.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/method.cc > CMakeFiles/xstream.dir/src/method.cc.i

CMakeFiles/xstream.dir/src/method.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/method.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/method.cc -o CMakeFiles/xstream.dir/src/method.cc.s

CMakeFiles/xstream.dir/src/method.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/method.cc.o.requires

CMakeFiles/xstream.dir/src/method.cc.o.provides: CMakeFiles/xstream.dir/src/method.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/method.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/method.cc.o.provides

CMakeFiles/xstream.dir/src/method.cc.o.provides.build: CMakeFiles/xstream.dir/src/method.cc.o


CMakeFiles/xstream.dir/src/simple_method.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/simple_method.cc.o: ../src/simple_method.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/xstream.dir/src/simple_method.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/simple_method.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/simple_method.cc

CMakeFiles/xstream.dir/src/simple_method.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/simple_method.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/simple_method.cc > CMakeFiles/xstream.dir/src/simple_method.cc.i

CMakeFiles/xstream.dir/src/simple_method.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/simple_method.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/simple_method.cc -o CMakeFiles/xstream.dir/src/simple_method.cc.s

CMakeFiles/xstream.dir/src/simple_method.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/simple_method.cc.o.requires

CMakeFiles/xstream.dir/src/simple_method.cc.o.provides: CMakeFiles/xstream.dir/src/simple_method.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/simple_method.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/simple_method.cc.o.provides

CMakeFiles/xstream.dir/src/simple_method.cc.o.provides.build: CMakeFiles/xstream.dir/src/simple_method.cc.o


CMakeFiles/xstream.dir/src/method_manager.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/method_manager.cc.o: ../src/method_manager.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/xstream.dir/src/method_manager.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/method_manager.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/method_manager.cc

CMakeFiles/xstream.dir/src/method_manager.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/method_manager.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/method_manager.cc > CMakeFiles/xstream.dir/src/method_manager.cc.i

CMakeFiles/xstream.dir/src/method_manager.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/method_manager.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/method_manager.cc -o CMakeFiles/xstream.dir/src/method_manager.cc.s

CMakeFiles/xstream.dir/src/method_manager.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/method_manager.cc.o.requires

CMakeFiles/xstream.dir/src/method_manager.cc.o.provides: CMakeFiles/xstream.dir/src/method_manager.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/method_manager.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/method_manager.cc.o.provides

CMakeFiles/xstream.dir/src/method_manager.cc.o.provides.build: CMakeFiles/xstream.dir/src/method_manager.cc.o


CMakeFiles/xstream.dir/src/timer/timer.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/timer/timer.cc.o: ../src/timer/timer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/xstream.dir/src/timer/timer.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/timer/timer.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/timer/timer.cc

CMakeFiles/xstream.dir/src/timer/timer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/timer/timer.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/timer/timer.cc > CMakeFiles/xstream.dir/src/timer/timer.cc.i

CMakeFiles/xstream.dir/src/timer/timer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/timer/timer.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/timer/timer.cc -o CMakeFiles/xstream.dir/src/timer/timer.cc.s

CMakeFiles/xstream.dir/src/timer/timer.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/timer/timer.cc.o.requires

CMakeFiles/xstream.dir/src/timer/timer.cc.o.provides: CMakeFiles/xstream.dir/src/timer/timer.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/timer/timer.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/timer/timer.cc.o.provides

CMakeFiles/xstream.dir/src/timer/timer.cc.o.provides.build: CMakeFiles/xstream.dir/src/timer/timer.cc.o


CMakeFiles/xstream.dir/src/node.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/node.cc.o: ../src/node.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/xstream.dir/src/node.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/node.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/node.cc

CMakeFiles/xstream.dir/src/node.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/node.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/node.cc > CMakeFiles/xstream.dir/src/node.cc.i

CMakeFiles/xstream.dir/src/node.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/node.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/node.cc -o CMakeFiles/xstream.dir/src/node.cc.s

CMakeFiles/xstream.dir/src/node.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/node.cc.o.requires

CMakeFiles/xstream.dir/src/node.cc.o.provides: CMakeFiles/xstream.dir/src/node.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/node.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/node.cc.o.provides

CMakeFiles/xstream.dir/src/node.cc.o.provides.build: CMakeFiles/xstream.dir/src/node.cc.o


CMakeFiles/xstream.dir/src/scheduler.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/scheduler.cc.o: ../src/scheduler.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/xstream.dir/src/scheduler.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/scheduler.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/scheduler.cc

CMakeFiles/xstream.dir/src/scheduler.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/scheduler.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/scheduler.cc > CMakeFiles/xstream.dir/src/scheduler.cc.i

CMakeFiles/xstream.dir/src/scheduler.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/scheduler.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/scheduler.cc -o CMakeFiles/xstream.dir/src/scheduler.cc.s

CMakeFiles/xstream.dir/src/scheduler.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/scheduler.cc.o.requires

CMakeFiles/xstream.dir/src/scheduler.cc.o.provides: CMakeFiles/xstream.dir/src/scheduler.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/scheduler.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/scheduler.cc.o.provides

CMakeFiles/xstream.dir/src/scheduler.cc.o.provides.build: CMakeFiles/xstream.dir/src/scheduler.cc.o


CMakeFiles/xstream.dir/src/xthread.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/xthread.cc.o: ../src/xthread.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/xstream.dir/src/xthread.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/xthread.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xthread.cc

CMakeFiles/xstream.dir/src/xthread.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/xthread.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xthread.cc > CMakeFiles/xstream.dir/src/xthread.cc.i

CMakeFiles/xstream.dir/src/xthread.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/xthread.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xthread.cc -o CMakeFiles/xstream.dir/src/xthread.cc.s

CMakeFiles/xstream.dir/src/xthread.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/xthread.cc.o.requires

CMakeFiles/xstream.dir/src/xthread.cc.o.provides: CMakeFiles/xstream.dir/src/xthread.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/xthread.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/xthread.cc.o.provides

CMakeFiles/xstream.dir/src/xthread.cc.o.provides.build: CMakeFiles/xstream.dir/src/xthread.cc.o


CMakeFiles/xstream.dir/src/thread_manager.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/thread_manager.cc.o: ../src/thread_manager.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/xstream.dir/src/thread_manager.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/thread_manager.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/thread_manager.cc

CMakeFiles/xstream.dir/src/thread_manager.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/thread_manager.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/thread_manager.cc > CMakeFiles/xstream.dir/src/thread_manager.cc.i

CMakeFiles/xstream.dir/src/thread_manager.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/thread_manager.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/thread_manager.cc -o CMakeFiles/xstream.dir/src/thread_manager.cc.s

CMakeFiles/xstream.dir/src/thread_manager.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/thread_manager.cc.o.requires

CMakeFiles/xstream.dir/src/thread_manager.cc.o.provides: CMakeFiles/xstream.dir/src/thread_manager.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/thread_manager.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/thread_manager.cc.o.provides

CMakeFiles/xstream.dir/src/thread_manager.cc.o.provides.build: CMakeFiles/xstream.dir/src/thread_manager.cc.o


CMakeFiles/xstream.dir/src/xthread_pool.cc.o: CMakeFiles/xstream.dir/flags.make
CMakeFiles/xstream.dir/src/xthread_pool.cc.o: ../src/xthread_pool.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/xstream.dir/src/xthread_pool.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xstream.dir/src/xthread_pool.cc.o -c /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xthread_pool.cc

CMakeFiles/xstream.dir/src/xthread_pool.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xstream.dir/src/xthread_pool.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xthread_pool.cc > CMakeFiles/xstream.dir/src/xthread_pool.cc.i

CMakeFiles/xstream.dir/src/xthread_pool.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xstream.dir/src/xthread_pool.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/src/xthread_pool.cc -o CMakeFiles/xstream.dir/src/xthread_pool.cc.s

CMakeFiles/xstream.dir/src/xthread_pool.cc.o.requires:

.PHONY : CMakeFiles/xstream.dir/src/xthread_pool.cc.o.requires

CMakeFiles/xstream.dir/src/xthread_pool.cc.o.provides: CMakeFiles/xstream.dir/src/xthread_pool.cc.o.requires
	$(MAKE) -f CMakeFiles/xstream.dir/build.make CMakeFiles/xstream.dir/src/xthread_pool.cc.o.provides.build
.PHONY : CMakeFiles/xstream.dir/src/xthread_pool.cc.o.provides

CMakeFiles/xstream.dir/src/xthread_pool.cc.o.provides.build: CMakeFiles/xstream.dir/src/xthread_pool.cc.o


# Object files for target xstream
xstream_OBJECTS = \
"CMakeFiles/xstream.dir/src/common/com_func.cc.o" \
"CMakeFiles/xstream.dir/src/profiler.cc.o" \
"CMakeFiles/xstream.dir/src/xstream.cc.o" \
"CMakeFiles/xstream.dir/src/xstream_config.cc.o" \
"CMakeFiles/xstream.dir/src/method.cc.o" \
"CMakeFiles/xstream.dir/src/simple_method.cc.o" \
"CMakeFiles/xstream.dir/src/method_manager.cc.o" \
"CMakeFiles/xstream.dir/src/timer/timer.cc.o" \
"CMakeFiles/xstream.dir/src/node.cc.o" \
"CMakeFiles/xstream.dir/src/scheduler.cc.o" \
"CMakeFiles/xstream.dir/src/xthread.cc.o" \
"CMakeFiles/xstream.dir/src/thread_manager.cc.o" \
"CMakeFiles/xstream.dir/src/xthread_pool.cc.o"

# External object files for target xstream
xstream_EXTERNAL_OBJECTS =

libxstream.so: CMakeFiles/xstream.dir/src/common/com_func.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/profiler.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/xstream.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/xstream_config.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/method.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/simple_method.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/method_manager.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/timer/timer.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/node.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/scheduler.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/xthread.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/thread_manager.cc.o
libxstream.so: CMakeFiles/xstream.dir/src/xthread_pool.cc.o
libxstream.so: CMakeFiles/xstream.dir/build.make
libxstream.so: CMakeFiles/xstream.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking CXX shared library libxstream.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xstream.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xstream.dir/build: libxstream.so

.PHONY : CMakeFiles/xstream.dir/build

CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/common/com_func.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/profiler.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/xstream.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/xstream_config.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/method.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/simple_method.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/method_manager.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/timer/timer.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/node.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/scheduler.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/xthread.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/thread_manager.cc.o.requires
CMakeFiles/xstream.dir/requires: CMakeFiles/xstream.dir/src/xthread_pool.cc.o.requires

.PHONY : CMakeFiles/xstream.dir/requires

CMakeFiles/xstream.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xstream.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xstream.dir/clean

CMakeFiles/xstream.dir/depend:
	cd /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/CMakeFiles/xstream.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xstream.dir/depend

