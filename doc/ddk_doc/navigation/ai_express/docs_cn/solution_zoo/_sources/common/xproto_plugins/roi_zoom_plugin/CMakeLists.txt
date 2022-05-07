cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
option(RELEASE_LIB "build version of release" ON)

# 项目名称
project(roi_zoom_plugin)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
# 添加头文件路径
include_directories(
  include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/xproto/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../message/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/protobuf/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/opencv/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/libjpeg-turbo/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common
)

link_directories(
  ${HOST_PACKAGE_DIR}/xproto/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/opencv/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/gtest/lib
)

# 头文件路径信息
file(GLOB PROJECT_INC
    "include/roi_zoom_plugin/*.h"
    "include/utils/*.h"
)

# 源文件路径信息
file(GLOB PROJECT_SRC
    "src/roi_zoom_plugin/*.cc"
    "src/utils/*.cc"
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

set(BASE_LIBRARIES opencv_world pthread jsoncpp hobotlog)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/configs
    DESTINATION ${MY_OUTPUT_ROOT}/configs)

