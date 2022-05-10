cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "build subdirectory from here" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
# 项目名称
project(uvc_server_plugin)
set(CMAKE_CXX_STANDARD 11)
option(RELEASE_LIB "build version of release" ON)

STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}) 

set(SOLUTION_ZOO_COMMON_DIR)
  function(get_parent_dir arg dir n)
    foreach(i RANGE ${ARGV2})
      message("i:${i} dir:${dir}")
      get_filename_component(dir ${dir} DIRECTORY)
    endforeach(i)
    set(SOLUTION_ZOO_COMMON_DIR ${dir} PARENT_SCOPE)
  endfunction()
get_parent_dir(XSTREAM_CURRENT_DIR ${PROJECT_SOURCE_DIR} 1)

message("solution_zoo common dir:${SOLUTION_ZOO_COMMON_DIR}")
# get deps directory end

set(WRAPPER_MEDIA_CODEC_SOURCE ${SOLUTION_ZOO_COMMON_DIR}/wrapper/media_codec/)
message("media codec wrapper dir:${WRAPPER_MEDIA_CODEC_SOURCE}")

if (NOT TARGET xstream_media_codec)
add_subdirectory(${WRAPPER_MEDIA_CODEC_SOURCE}
  ${WRAPPER_MEDIA_CODEC_SOURCE}/output
)
endif()

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
# 添加头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/zeroMQ/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/blocking_queue/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/xproto_plugins/message/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/wrapper/media_codec/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/thread_pool/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/singleton/
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio
  ${HOST_PACKAGE_DIR}/uvc_server/include
  ${HOST_PACKAGE_DIR}/xproto/include
  ${HOST_PACKAGE_DIR}/xstream/include
)

link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/protobuf/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/zeroMQ/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/uvc_server/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia
)

file(GLOB PROJECT_INC
    "include/*.h"
    "include/uvc_server_plugin/*.h"
)

file(GLOB PROJECT_SRC 
    "src/*.cc"
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})

set(BASE_LIBRARIES pthread jsoncpp protobuf
      hbmedia uvc_server zmq xstream_media_codec)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include/)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/configs/
    DESTINATION ${MY_OUTPUT_ROOT}/config/)
