cmake_minimum_required(VERSION 2.8)
project(00_quick_start)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

if (NOT DEPS_ROOT)
    set(DEPS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../deps/aarch64)
endif ()
message(STATUS "DEPS_ROOT: ${DEPS_ROOT}")

if (NOT RELEASE_ROOT)
    set(RELEASE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../script/)
endif ()
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
        ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
        ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
        ${DEPS_ROOT}/glog/lib
        ${DEPS_ROOT}/gflags/lib
        ${DEPS_ROOT}/opencv/lib)

SET(LINK_libs dnn gflags glog opencv_world zlib dl rt pthread dnn)

add_executable(run_mobileNetV1_224x224 src/run_mobileNetV1_224x224.cc)
target_link_libraries(run_mobileNetV1_224x224 ${LINK_libs})

install(TARGETS run_mobileNetV1_224x224 DESTINATION ${RELEASE_BIN_DIR})

file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so DESTINATION ${RELEASE_LIB_DIR})
file(COPY ${DEPS_ROOT}/opencv/lib/libopencv_world.so.3.4 DESTINATION ${RELEASE_LIB_DIR})
file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libhbrt_bernoulli_aarch64.so DESTINATION ${RELEASE_LIB_DIR})
