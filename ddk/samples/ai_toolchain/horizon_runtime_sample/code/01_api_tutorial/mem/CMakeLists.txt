cmake_minimum_required(VERSION 2.8)
project(mem)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

if(NOT DEPS_ROOT)
        set(DEPS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/aarch64)
endif()
message(STATUS "DEPS_ROOT: ${DEPS_ROOT}")

if(NOT RELEASE_ROOT)
        set(RELEASE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../script/)
endif()
message(STATUS "RELEASE_ROOT: ${RELEASE_ROOT}")

set(RELEASE_LIB_DIR ${RELEASE_ROOT}/aarch64/lib/)
set(RELEASE_BIN_DIR ${RELEASE_ROOT}/aarch64/bin/)

include_directories(
        ${HOST_PACKAGE_DIR}/dnn/include)

link_directories(
    ${HOST_PACKAGE_DIR}/dnn/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu)

add_executable(sys_mem_example src/sys_mem_example.cc)
target_link_libraries(sys_mem_example dnn dl rt pthread)

install(TARGETS sys_mem_example DESTINATION ${RELEASE_BIN_DIR}/)
file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so DESTINATION ${RELEASE_LIB_DIR})
