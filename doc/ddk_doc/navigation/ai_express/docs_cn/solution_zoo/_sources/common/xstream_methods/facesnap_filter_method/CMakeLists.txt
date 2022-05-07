cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

project(facesnap_filter_method)

message("${PROJECT_SOURCE_DIR}")
message("config types: ${CMAKE_CONFIGURATION_TYPES}")

option(RELEASE_LIB "build version of release" ON)
if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
else ()
    set(CMAKE_BUILD_TYPE Debug)
endif ()

message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}" )

option(USE_ENCRYPT "whether to use hobot aes encrypt" OFF)
if (NOT ${USE_ENCRYPT})
    add_definitions(-DNO_ENCRYPT)
    message("not use hobot aes encrypt")
else ()
    message("use hobot aes encrypt")
endif()

option(USE_OMP "whether to use omp" OFF)
if (NOT ${USE_OMP})
    add_definitions(-DNO_OMP)
    message("not use omp..")
else ()
    message("use omp..")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp" )
endif ()

option(IPC_VER "whether to use omp" OFF)
if (NOT ${IPC_VER})
    add_definitions(-DNOIPC_VER)
    message("not ipc ver..")
else ()
    message("ipc ver..")
    add_definitions(-DUSE_ZLOG)
    message("use zlog....")
    add_definitions(-DNO_OMP)
    message("not use omp....")
endif ()

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
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/include
  ${HOST_PACKAGE_DIR}/xstream/include
)
link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/lib
)

set(SOURCE_FILES
        src/facesnap_filter_method.cc
)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

add_subdirectory(test)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
