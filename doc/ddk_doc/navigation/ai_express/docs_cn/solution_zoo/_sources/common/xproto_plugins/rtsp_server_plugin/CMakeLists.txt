cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURDIR ${CMAKE_CURRENT_SOURCE_DIR})
message("${CMAKE_CURRENT_SOURCE_DIR}")
# 项目名称
project(rtsp_server_plugin)
option(X3_MEDIA_CODEC "x3 media codec" ON)
if (${X3_MEDIA_CODEC})
    add_definitions(-DX3_MEDIA_CODEC)
endif()

message(${PROJECT_SOURCE_DIR})
set(SOLUTION_ZOO_COMMON_DIR)
  function(get_parent_dir arg dir n)
    foreach(i RANGE ${ARGV2})
      # message("i:${i} dir:${dir}")
      get_filename_component(dir ${dir} DIRECTORY)
    endforeach(i)
    set(SOLUTION_ZOO_COMMON_DIR ${dir} PARENT_SCOPE)
  endfunction()
get_parent_dir(XSTREAM_CURRENT_DIR ${PROJECT_SOURCE_DIR} 1)

message("solution_zoo common dir:${SOLUTION_ZOO_COMMON_DIR}")
# get deps directory end

set(WRAPPER_CODEC_SOURCE ${SOLUTION_ZOO_COMMON_DIR}/wrapper/media_codec/)
message("meida wrapper dir:${WRAPPER_CODEC_SOURCE}")

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
# 添加头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/xproto_plugins/message/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/xproto_plugins/protobuf/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/protobuf/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/wrapper/media_codec/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio/
  ${HOST_PACKAGE_DIR}/rtsp_server/include/
  ${HOST_PACKAGE_DIR}/xproto/include
  ${HOST_PACKAGE_DIR}/xstream/include
)

link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/protobuf/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
  ${HOST_PACKAGE_DIR}/rtsp_server/lib/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/wrapper/media_codec/output/xstream_media_codec/lib/
)

file(GLOB PROJECT_INC
    "include/*.h"
)
file(GLOB PROJECT_SRC "src/*.cc")

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})

if (NOT TARGET xstream_media_codec)
add_subdirectory(${WRAPPER_CODEC_SOURCE}
  ${WRAPPER_CODEC_SOURCE}/output
)
endif()

set(BASE_LIBRARIES pthread jsoncpp protobuf 
)

set(LIVE_LIB rtsp_server)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} 
  xstream_media_codec
  ${BASE_LIBRARIES}
  ${LIVE_LIB}
)

#add_subdirectory(example)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/rtsp_server_plugin)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
# install configs, some scripts will depends this
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/configs/
    DESTINATION ${MY_OUTPUT_ROOT}/config/)
