cmake_minimum_required(VERSION 2.8)
project(ai_benchmarks)

set(DEPS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/deps/aarch64)
set(RELEASE_ROOT ${CMAKE_INSTALL_PREFIX})

message(STATUS "DEPS_ROOT: ${DEPS_ROOT}")
message(STATUS "RELEASE_ROOT: ${RELEASE_ROOT}")

set(RELEASE_LIB_DIR ${RELEASE_ROOT}/aarch64/lib/)
set(RELEASE_BIN_DIR ${RELEASE_ROOT}/aarch64/bin/)

if (XJ3)
    if (DEFINED ENV{HORIZON_LIB_PATH})
        set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
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
message(STATUS "HOST_PACKAGE_DIR: ${HOST_PACKAGE_DIR}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
        ${HOST_PACKAGE_DIR}/dnn/include
        ${DEPS_ROOT}/glog/include
        ${DEPS_ROOT}/gflags/include
        ${DEPS_ROOT}/rapidjson
        ${DEPS_ROOT}/opencv/include
        ${HOST_PACKAGE_DIR}/xproto/include
        ${HOST_PACKAGE_DIR}/xstream/include
)

link_directories(
        ${HOST_PACKAGE_DIR}/dnn/lib
        ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
        ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
        ${DEPS_ROOT}/glog/lib
        ${DEPS_ROOT}/gflags/lib
        ${DEPS_ROOT}/opencv/lib
        ${HOST_PACKAGE_DIR}/xproto/lib
        ${HOST_PACKAGE_DIR}/xstream/lib
)

SET(Link_libs
        dnn
        cnn_intf
        hbrt_bernoulli_aarch64
        gflags
        glog
        opencv_world
        dl
        rt
        xproto
        xstream
        pthread)


file(GLOB INPUT_SOURCE src/input/*.cc)
file(GLOB OUTPUT_SOURCE src/output/*.cc)
file(GLOB METHOD_SOURCE src/method/*.cc)
file(GLOB PLUGIN_SOURCE src/plugin/*.cc)

set(UTILS_SOURCE
        src/utils/image_utils.cc
        src/utils/data_transformer.cc
        src/utils/nms.cc
        src/utils/stop_watch.cc
        src/utils/tensor_utils.cc
        src/utils/utils.cc
        src/utils/image_cache.cc
        )

add_executable(example
        src/simple_example.cc
        ${INPUT_SOURCE}
        ${OUTPUT_SOURCE}
        ${UTILS_SOURCE}
        ${METHOD_SOURCE}
        ${PLUGIN_SOURCE}
        )

target_link_libraries(example ${Link_libs})

install(TARGETS example
        DESTINATION ${RELEASE_BIN_DIR}/)
install(FILES ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so
        DESTINATION ${RELEASE_LIB_DIR})
install(FILES ${HOST_PACKAGE_DIR}/dnn/lib/libhbrt_bernoulli_aarch64.so
        DESTINATION ${RELEASE_LIB_DIR})
install(FILES ${DEPS_ROOT}/opencv/lib/libopencv_world.so.3.4
        DESTINATION ${RELEASE_LIB_DIR})
install(FILES ${HOST_PACKAGE_DIR}/xproto/lib/libxproto.so
        DESTINATION ${RELEASE_LIB_DIR})
install(FILES ${HOST_PACKAGE_DIR}/xstream/lib/libxstream.so
        DESTINATION ${RELEASE_LIB_DIR})
