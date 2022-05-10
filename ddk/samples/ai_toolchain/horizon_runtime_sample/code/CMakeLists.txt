cmake_minimum_required(VERSION 2.8)

project(dnn_tutorial)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

set(DEPS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/deps/aarch64)

if (XJ3)
    if (DEFINED ENV{HORIZON_LIB_PATH})
        set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64/)
    else()
        set(HOST_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/aarch64/xj3)
    endif()
elseif(J5)
    if (DEFINED ENV{HORIZON_LIB_PATH})
        set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/j5_aarch64)
    else()
        set(HOST_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/aarch64/j5)
    endif()
else()
    set(HOST_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/aarch64/j5)
endif ()

message(STATUS "Install directory: ${CMAKE_INSTALL_PREFIX}")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE " -O3 ")
set(CMAKE_C_FLAGS_RELEASE " -O3 ")

set(CMAKE_BUILD_TYPE ${build_type})
set(RELEASE_ROOT ${CMAKE_INSTALL_PREFIX})

set(RELEASE_LIB_DIR ${RELEASE_ROOT}/aarch64/lib/)
set(RELEASE_BIN_DIR ${RELEASE_ROOT}/aarch64/bin/)

add_subdirectory(00_quick_start)
add_subdirectory(01_api_tutorial)
add_subdirectory(02_advanced_samples)
add_subdirectory(03_misc)

file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so
        DESTINATION ${RELEASE_LIB_DIR})

file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libhbrt_bernoulli_aarch64.so
        DESTINATION ${RELEASE_LIB_DIR})

file(COPY ${DEPS_ROOT}/opencv/lib/libopencv_world.so.3.4
        DESTINATION ${RELEASE_LIB_DIR})
