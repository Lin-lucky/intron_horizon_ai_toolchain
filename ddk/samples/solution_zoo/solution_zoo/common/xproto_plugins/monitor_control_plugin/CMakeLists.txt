cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

option(RELEASE_LIB "build version of release" ON)

# 项目名称
project(monitor_control_plugin)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
# 添加头文件路径
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${HOST_PACKAGE_DIR}/xproto/include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio/
  ${PROJECT_SOURCE_DIR}/../../blocking_queue
  ${PROJECT_SOURCE_DIR}/../../singleton
  ${PROJECT_SOURCE_DIR}/../../thread_pool/
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/include
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/include
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/include
  ${PROJECT_SOURCE_DIR}/../../deps/protobuf/include
  ${PROJECT_SOURCE_DIR}/../../deps/zeroMQ/include
  ${PROJECT_SOURCE_DIR}/../../wrapper/media_codec/include
  ${PROJECT_SOURCE_DIR}/../message/include
  ${PROJECT_SOURCE_DIR}/../media_codec/include
)

# 头文件路径信息
file(GLOB PROJECT_INC
    "include/monitor_control_plugin/*.h"
    "include/utils/*.h"
    "include/utils/*.hpp"
)

# 源文件路径信息
file(GLOB PROJECT_SRC
    "src/monitor_control_plugin/*.cc"
    "src/utils/*.cpp"
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

set(BASE_LIBRARIES opencv_world pthread xproto_msgtype jsoncpp)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
# install configs, some scripts will depends this
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/configs/
    DESTINATION ${MY_OUTPUT_ROOT}/config/)

