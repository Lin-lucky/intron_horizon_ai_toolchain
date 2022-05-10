cmake_minimum_required(VERSION 2.8)
project(resize)

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
        ${HOST_PACKAGE_DIR}/dnn/include
        ${DEPS_ROOT}/glog/include
        ${DEPS_ROOT}/gflags/include
        ${DEPS_ROOT}/opencv/include)

link_directories(
    ${HOST_PACKAGE_DIR}/dnn/lib
    ${DEPS_ROOT}/glog/lib
    ${DEPS_ROOT}/gflags/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
    ${DEPS_ROOT}/opencv/lib)

SET(LINK_libs dnn gflags glog rt dl pthread opencv_world)


add_executable(resize_y_example src/resize_y_example.cc)
target_link_libraries(resize_y_example ${LINK_libs})

add_executable(resize_bgr_example src/resize_bgr_example.cc)
target_link_libraries(resize_bgr_example ${LINK_libs})

install(TARGETS resize_y_example DESTINATION ${RELEASE_BIN_DIR}/)
install(TARGETS resize_bgr_example DESTINATION ${RELEASE_BIN_DIR}/)

file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so DESTINATION ${RELEASE_LIB_DIR})
file(COPY ${DEPS_ROOT}/opencv/lib/libopencv_world.so.3.4 DESTINATION ${RELEASE_LIB_DIR})
