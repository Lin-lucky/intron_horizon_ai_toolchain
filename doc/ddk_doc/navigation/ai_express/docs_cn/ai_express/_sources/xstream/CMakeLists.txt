cmake_minimum_required(VERSION 2.8)
project(xstream
	LANGUAGES CXX
)

# PARENT_BUILD:OFF buid xstream separately
# PARENT_BUILD:ON git-repo build all
option(PARENT_BUILD "is build from parent" OFF)
option(INSTALL_TUTORIALS "is install tutorials " OFF)
option(BUILD_ALL_ARCH "is build all architecture " OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT ${PARENT_BUILD})
	get_filename_component(XSTREAM_CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
	# message("xstream current dir init:${XSTREAM_CURRENT_DIR}")
	set(RELEASE_ROOT_DIR)
	function(get_parent_dir arg dir n)
		foreach(i RANGE ${ARGV2})
			get_filename_component(dir ${dir} DIRECTORY)
			#message("i:${i} dir:${dir}")
		endforeach(i)
		set(RELEASE_ROOT_DIR ${dir} PARENT_SCOPE)
	endfunction()

	get_parent_dir(XSTREAM_CURRENT_DIR ${XSTREAM_CURRENT_DIR} 2)
	# message("xstream current dir:${XSTREAM_CURRENT_DIR}")
	# message("source dir:${RELEASE_ROOT_DIR}")

	option(X86_ARCH "x86 architecture" OFF)
	option(X86_CENTOS "centos release" OFF)

	# build separately install directory
  set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output)
endif()

option(RELEASE_LIB "build version of release" ON)

if (NOT ${PARENT_BUILD})
	if (${RELEASE_LIB})
			set(CMAKE_BUILD_TYPE Release)
			message(STATUS "BUILD TYPE:Release")
	else ()
			set(CMAKE_BUILD_TYPE Debug)
			set(HOBOT_BUILD_DEBUG "TRUE")
			message(STATUS "BUILD TYPE:Debug")
	endif ()

	list(APPEND CMAKE_C_FLAGS "-Wall -Wno-unknown-pragmas -fPIC -fexceptions -Werror ")
	if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
			list(APPEND CMAKE_C_FLAGS "-O3")
			set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC ")
	else ()
			list(APPEND CMAKE_C_FLAGS "-g -O0 -fPIC ")
	endif()

	# x86 architecture
	if (${X86_ARCH})
		if (${X86_CENTOS})
			message("tool chain:${CMAKE_CXX_COMPILER}")
			add_definitions(-std=c++11)
			set(CENTOS_THIRD_PARTY_DIR ${XSTREAM_CURRENT_DIR}/xstream/third_party/x86/centos/)
			include_directories(
				${CENTOS_THIRD_PARTY_DIR}jsoncpp/include/
				${CENTOS_THIRD_PARTY_DIR}hobotlog/include/
				${CENTOS_THIRD_PARTY_DIR}gtest/include/
			)
			link_directories(
				${CENTOS_THIRD_PARTY_DIR}jsoncpp/lib/
				${CENTOS_THIRD_PARTY_DIR}hobotlog/lib/
				${CENTOS_THIRD_PARTY_DIR}gtest/lib/
			)
		else ()
			message("tool chain:${CMAKE_CXX_COMPILER}")
			add_definitions(-std=c++11)
			set(UBUNTU_THIRD_PARTY_DIR ${XSTREAM_CURRENT_DIR}/xstream/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/)
			include_directories(
				${UBUNTU_THIRD_PARTY_DIR}jsoncpp/include/
				${UBUNTU_THIRD_PARTY_DIR}hobotlog/include/
				${UBUNTU_THIRD_PARTY_DIR}gtest/include/
				${UBUNTU_THIRD_PARTY_DIR}opencv/include/
			)
			link_directories(
				${UBUNTU_THIRD_PARTY_DIR}jsoncpp/lib/
				${UBUNTU_THIRD_PARTY_DIR}hobotlog/lib/
				${UBUNTU_THIRD_PARTY_DIR}gtest/lib/
			)
		endif ()
	else ()
		set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
		message("tool chain:${CMAKE_CXX_COMPILER}")
		set(AARCH64_THIRD_PARTY_DIR ${XSTREAM_CURRENT_DIR}/xstream/third_party/aarch64/)
		include_directories(
			${RELEASE_ROOT_DIR}/deps/jsoncpp/include/
			${RELEASE_ROOT_DIR}/deps/hobotlog/include/
			${AARCH64_THIRD_PARTY_DIR}jsoncpp/include/
			${AARCH64_THIRD_PARTY_DIR}hobotlog/include/
			${AARCH64_THIRD_PARTY_DIR}opencv/include/
			${AARCH64_THIRD_PARTY_DIR}gtest/include/
		)
		link_directories(
				${AARCH64_THIRD_PARTY_DIR}hobotlog/lib
				${AARCH64_THIRD_PARTY_DIR}jsoncpp/lib
				${AARCH64_THIRD_PARTY_DIR}gtest/lib
			)
	endif ()
endif ()

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/src/include
	${CMAKE_CURRENT_SOURCE_DIR}/src/include/common
	${CMAKE_CURRENT_SOURCE_DIR}/src/include/timer
)

include_directories(SYSTEM /usr/include/python2.7)

set(SOURCE_FILES
	src/common/com_func.cc
	src/profiler.cc
	src/xstream.cc
	src/xstream_config.cc
	src/method.cc
	src/simple_method.cc
)

set(NATIVE_ENGINE_SOURCES
	src/method_manager.cc
	src/timer/timer.cc
	src/node.cc
	src/scheduler.cc
	src/xthread.cc
	src/thread_manager.cc
	src/xthread_pool.cc
)
set(SOURCE_FILES ${SOURCE_FILES} ${NATIVE_ENGINE_SOURCES})

option(BUILD_SHARED_LIBS "build shared libs" ON)
if (${BUILD_SHARED_LIBS} AND ${RELEASE_LIB})
set(CMAKE_SHARED_LINKER_FLAGS " -Wl,--version-script=${PROJECT_SOURCE_DIR}/export_lib.lst")

endif()
add_library(xstream ${SOURCE_FILES})

target_link_libraries(xstream
	PUBLIC rt)

target_link_libraries(xstream
  PUBLIC jsoncpp hobotlog)

add_subdirectory(test)
add_subdirectory(benchmark)

add_subdirectory(tutorials/stage1_hello_world)
add_subdirectory(tutorials/stage2_multithread)
add_subdirectory(tutorials/stage3_update_parameter)
add_subdirectory(tutorials/stage4_multisource)
add_subdirectory(tutorials/stage5_timeout_alarm)
add_subdirectory(tutorials/stage6_profile)
add_subdirectory(tutorials/stage7_node_output)
add_subdirectory(tutorials/stage8_sub_workflow)
add_subdirectory(tutorials/stage9_disable_method)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xstream
  DESTINATION ${MY_OUTPUT_ROOT}/include/
)

if (${BUILD_ALL_ARCH})
  if(${X86_ARCH})
    # centos
    if (${X86_CENTOS})
      # .so
      if (${BUILD_SHARED_LIBS})
        # release
        if (${RELEASE_LIB})
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/centos/shared_lib/
          )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/centos/shared_lib/debug
          )
        endif()
      # .a
     if(DEFINED ENV{HORIZON_LIB_PATH})
      set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
    else()
      set(HOST_PACKAGE_DIR $ENV{HOME}/.horizon/ddk/xj3_aarch64)
    endif()
      else()
        # release
        if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/centos/static_lib/
        )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/centos/static_lib/debug
        )
        endif()
      endif()
    # ubuntu
    else ()
      # .so
      if (${BUILD_SHARED_LIBS})
        # release
        if (${RELEASE_LIB})
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/shared_lib/
          )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/shared_lib/debug
          )
        endif()
      # .a
      else()
        # release
        if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/static_lib/
        )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/static_lib/debug
           if(DEFINED ENV{HORIZON_LIB_PATH})
            set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
          else()
            set(HOST_PACKAGE_DIR $ENV{HOME}/.horizon/ddk/xj3_aarch64)
          endif()
        )
        endif()
      endif()
    endif()
  # xj3j5
  else()
    # .so
    if (${BUILD_SHARED_LIBS})
      # release
      if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/shared_lib/
        )
      # debug
      else ()
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/shared_lib/debug
        )
      endif()
    # .a
    else()
      # release
      if (${RELEASE_LIB})
      install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/static_lib/
      )
      # debug
      else ()
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/static_lib/debug
      )
      endif()
    endif()
  endif()
else()
  install(TARGETS ${PROJECT_NAME}
	  DESTINATION ${MY_OUTPUT_ROOT}/lib
  )
endif()

if (${INSTALL_TUTORIALS})
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md
    DESTINATION ${MY_OUTPUT_ROOT}/
  )

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tutorials
    DESTINATION ${MY_OUTPUT_ROOT}/
  )

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    DESTINATION ${MY_OUTPUT_ROOT}/tutorials
  )
endif()
