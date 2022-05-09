cmake_minimum_required(VERSION 2.8)

project(image_utils)

# PARENT_BUILD:OFF buid xstream separately
# PARENT_BUILD:ON git-repo build all
option(PARENT_BUILD "is build from parent" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT ${PARENT_BUILD})
	get_filename_component(IMAGEUTILSLIB_CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
	#message("xstream current dir init:${IMAGEUTILSLIB_CURRENT_DIR}")
	set(RELEASE_ROOT_DIR)
	function(get_parent_dir arg dir n)
		foreach(i RANGE ${ARGV2})
			get_filename_component(dir ${dir} DIRECTORY)
			#message("i:${i} dir:${dir}")
		endforeach(i)
		set(RELEASE_ROOT_DIR ${dir} PARENT_SCOPE)
	endfunction()

	get_parent_dir(IMAGEUTILSLIB_CURRENT_DIR ${IMAGEUTILSLIB_CURRENT_DIR} 2)
	message("xstream current dir:${IMAGEUTILSLIB_CURRENT_DIR}")
	message("source dir:${RELEASE_ROOT_DIR}")

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
			set(CENTOS_THIRD_PARTY_DIR ${IMAGEUTILSLIB_CURRENT_DIR}/image_utils/third_party/x86/centos/)
			include_directories(
				${CENTOS_THIRD_PARTY_DIR}/hobotlog/include/
				${CENTOS_THIRD_PARTY_DIR}/opencv/include/
				${CENTOS_THIRD_PARTY_DIR}/gtest/include/
				${CENTOS_THIRD_PARTY_DIR}/libjpeg-turbo/include/
				${CENTOS_THIRD_PARTY_DIR}/libyuv/include/
			)
			link_directories(
				${CENTOS_THIRD_PARTY_DIR}/hobotlog/lib
				${CENTOS_THIRD_PARTY_DIR}/opencv/lib/
				${CENTOS_THIRD_PARTY_DIR}/gtest/lib
				${CENTOS_THIRD_PARTY_DIR}/libjpeg-turbo/lib/
				${CENTOS_THIRD_PARTY_DIR}/libyuv/lib/
			)
		else ()
			message("tool chain:${CMAKE_CXX_COMPILER}")
			add_definitions(-std=c++11)
			set(UBUNTU_THIRD_PARTY_DIR ${IMAGEUTILSLIB_CURRENT_DIR}/image_utils/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/)
			include_directories(
				${UBUNTU_THIRD_PARTY_DIR}jsoncpp/include/
				${UBUNTU_THIRD_PARTY_DIR}hobotlog/include/
				${UBUNTU_THIRD_PARTY_DIR}opencv/include/
			)
			link_directories(
				${UBUNTU_THIRD_PARTY_DIR}jsoncpp/lib/
				${UBUNTU_THIRD_PARTY_DIR}hobotlog/lib
			)
		endif ()
	else ()
		set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
		message("tool chain:${CMAKE_CXX_COMPILER}")
		set(AARCH64_THIRD_PARTY_DIR ${IMAGEUTILSLIB_CURRENT_DIR}/image_utils/third_party/aarch64/)
		message("thrid_party_dir:${AARCH64_THIRD_PARTY_DIR}")
		include_directories(
			${AARCH64_THIRD_PARTY_DIR}/hobotlog/include/
			${AARCH64_THIRD_PARTY_DIR}/opencv/include/
			${AARCH64_THIRD_PARTY_DIR}/gtest/include/
			${AARCH64_THIRD_PARTY_DIR}/libjpeg-turbo/include/
			${AARCH64_THIRD_PARTY_DIR}/libyuv/include/
		)
		link_directories(
				${AARCH64_THIRD_PARTY_DIR}/hobotlog/lib
				${AARCH64_THIRD_PARTY_DIR}/opencv/lib/
				${AARCH64_THIRD_PARTY_DIR}/gtest/lib
				${AARCH64_THIRD_PARTY_DIR}/libjpeg-turbo/lib/
				${AARCH64_THIRD_PARTY_DIR}/libyuv/lib/
			)
	endif ()
endif ()

if(BGR2I420_WITH_LIBYUV)
        add_definitions(-DBGR2I420_WITH_LIBYUV)
endif(BGR2I420_WITH_LIBYUV)

include_directories(
        include
)
file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

option(BUILD_SHARED_LIBS "build shared libs" ON)
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} libturbojpeg.a libyuv.a
    libopencv_world.a liblibjasper.a liblibjpeg-turbo.a liblibpng.a liblibprotobuf.a liblibtiff.a
    liblibwebp.a libquirc.a libtegra_hal.a libzlib.a)

add_subdirectory(test)
add_subdirectory(example)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/image_utils/image_utils.h
        DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME}
				DESTINATION ${MY_OUTPUT_ROOT}/lib)
