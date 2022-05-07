cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(grading_method)

message("${PROJECT_SOURCE_DIR}")
option(RELEASE_LIB "build version of release" ON)
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()

message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}" )
# get deps directory begin
set(SOLUTION_ZOO_COMMON_DIR)
	function(get_parent_dir arg dir n)
		foreach(i RANGE ${ARGV2})
			get_filename_component(dir ${dir} DIRECTORY)
			# message("i:${i} dir:${dir}")
		endforeach(i)
		set(SOLUTION_ZOO_COMMON_DIR ${dir} PARENT_SCOPE)
	endfunction()
get_parent_dir(XSTREAM_CURRENT_DIR ${PROJECT_SOURCE_DIR} 1)

# message("solution_zoo common dir:${SOLUTION_ZOO_COMMON_DIR}")
# get deps directory end

set(CMAKE_CXX_STANDARD 11)

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -fPIC -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEBUG -g -O0 -fPIC")
endif()

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
include_directories(
  include
  ${SOLUTION_ZOO_COMMON_DIR}/deps/hobotlog/include/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/jsoncpp/include/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/gtest/include/
  ${HOST_PACKAGE_DIR}/xstream/include/
)

link_directories(
  /usr/local/cuda/lib64/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/hobotlog/lib/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/jsoncpp/lib/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/gtest/lib/
  ${HOST_PACKAGE_DIR}/xstream/lib/
)

set(BASE_LIBRARIES xstream hobotlog jsoncpp.a rt dl pthread)

set(SOURCE_FILES
  src/grading_method.cc
  src/weight_grading.cc
)

set(GRADING_LIBRARIES rt)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${GRADING_LIBRARIES}  ${BASE_LIBRARIES} )

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/grading_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

add_subdirectory(test)
