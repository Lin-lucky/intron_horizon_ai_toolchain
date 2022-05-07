cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(snapshot_method)

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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
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
  ${SOLUTION_ZOO_COMMON_DIR}/deps/opencv/include/
  ${HOST_PACKAGE_DIR}/xstream/include/
  ${HOST_PACKAGE_DIR}/image_utils/include/      
)

link_directories(
  /usr/local/cuda/lib64/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/hobotlog/lib/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/jsoncpp/lib/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/gtest/lib/
  ${SOLUTION_ZOO_COMMON_DIR}/deps/opencv/lib/
  ${HOST_PACKAGE_DIR}/xstream/lib/
  ${HOST_PACKAGE_DIR}/image_utils/lib/
)


#set(BASE_LIBRARIES image_utils libturbojpeg.a libyuv.a jsoncpp.a hobotlog rt dl pthread)
set(BASE_LIBRARIES image_utils jsoncpp.a hobotlog rt dl pthread)

set(SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/src/snapshot_method.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/strategy/crop.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/strategy/first_num_best.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/snapshot_utils.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/snapshot_data_type.cc
)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES include/snapshot_method/snapshot_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

# add_subdirectory(test)
