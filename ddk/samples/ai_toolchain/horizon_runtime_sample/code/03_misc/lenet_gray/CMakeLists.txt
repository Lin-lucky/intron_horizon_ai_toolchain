cmake_minimum_required(VERSION 2.8)
project(lenet_gray)

if(NOT DEPS_ROOT)
        set(DEPS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/aarch64)
endif()
message(STATUS "DEPS_ROOT: ${DEPS_ROOT}")

if(NOT RELEASE_ROOT)
        set(RELEASE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../script/)
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
        ${DEPS_ROOT}/opencv/lib
        ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
        ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu)

SET(LINK_libs dnn gflags glog opencv_world zlib rt dl pthread dnn)

add_executable(run_lenet_gray src/run_lenet_gray.cc)
target_link_libraries(run_lenet_gray ${LINK_libs})

install(TARGETS run_lenet_gray DESTINATION ${RELEASE_BIN_DIR})

file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so DESTINATION ${RELEASE_LIB_DIR})
file(COPY ${DEPS_ROOT}/opencv/lib/libopencv_world.so.3.4 DESTINATION ${RELEASE_LIB_DIR})
file(COPY ${HOST_PACKAGE_DIR}/dnn/lib/libhbrt_bernoulli_aarch64.so DESTINATION ${RELEASE_LIB_DIR})
