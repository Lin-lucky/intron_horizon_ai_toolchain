cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
option(RELEASE_LIB "build version of release" ON)

project(fasterrcnn_method)

message("${PROJECT_SOURCE_DIR}")
if (NOT (${RELEASE_LIB}))
  list(APPEND CMAKE_C_FLAGS " -g -Og")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11 ")

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
include_directories(
  include
  ${HOST_PACKAGE_DIR}/bpu_predict/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/image_utils/include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/include
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/include
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/include
  ${PROJECT_SOURCE_DIR}/../../deps
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/include
)

link_directories(
  ${CMAKE_SOURCE_DIR}/lib/
  ${HOST_PACKAGE_DIR}/bpu_predict/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/image_utils/lib
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/lib
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/lib
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/lib
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/lib
)

set(SOURCE_FILES
        src/fasterrcnn_method.cc
        src/fasterrcnn_imp.cc
        src/yuv_utils.cc
        src/dump.cc
        )

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

#set(DYNAMIC_DEPS_LIBS xstream-imagetools libturbojpeg.a libyuv.a bpu_predict cnn_intf hbrt_bernoulli_aarch64 cam vio)
set(DYNAMIC_DEPS_LIBS image_utils bpu_predict cnn_intf hbrt_bernoulli_aarch64)
target_link_libraries(${PROJECT_NAME} PRIVATE ${DYNAMIC_DEPS_LIBS})

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${PROJECT_SOURCE_DIR}/include/fasterrcnn_method/fasterrcnn_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/fasterrcnn_method)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/configs
        DESTINATION ${MY_OUTPUT_ROOT}/)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

add_subdirectory(example)
add_subdirectory(test)