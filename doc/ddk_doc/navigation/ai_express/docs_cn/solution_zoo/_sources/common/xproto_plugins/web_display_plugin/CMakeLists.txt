cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "build subdirectory from here" OFF)
if(NOT ${PARENT_BUILD})
set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

# 项目名称
project(web_display_plugin)
set(CMAKE_CXX_STANDARD 11)
option(RELEASE_LIB "build version of release" ON)
option(X3_MEDIA_CODEC "x3 media codec" ON)
if (${X3_MEDIA_CODEC})
    add_definitions(-DX3_MEDIA_CODEC)
endif()
file(GLOB PROJECT_INC
    "include/web_display_plugin/*.h"
    "include/web_display_plugin/message/*.h"
    "include/web_display_plugin/server/*.h"
    "include/web_display_plugin/utils/*.h"
)

file(GLOB PROJECT_SRC
    "src/*.cc"
    "src/server/*.cc"
    "src/utils/*.cc"
    "src/web_display_plugin/*.cc"
    ${PROJECT_SOURCE_DIR}/../message/include/protobuf/x3.pb.cc
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})

include_directories(
    include
    src
)

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

set(WRAPPER_CODEC_SOURCE ${SOLUTION_ZOO_COMMON_DIR}/wrapper/media_codec)

if (NOT TARGET xstream_media_codec)
add_subdirectory(${WRAPPER_CODEC_SOURCE}
  ${WRAPPER_CODEC_SOURCE}/output
)
endif()

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
# 添加头文件路径
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/xproto/include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio
  ${PROJECT_SOURCE_DIR}/../..
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/include
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/include
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/include
  ${PROJECT_SOURCE_DIR}/../../deps
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/include
  ${PROJECT_SOURCE_DIR}/../../deps/uWS/include
  ${PROJECT_SOURCE_DIR}/../../deps/zlib/include
  ${PROJECT_SOURCE_DIR}/../../deps/protobuf/include
  ${PROJECT_SOURCE_DIR}/../../wrapper/media_codec/include
  ${PROJECT_SOURCE_DIR}/../message/include
  ${PROJECT_SOURCE_DIR}/../media_codec/include
  ${PROJECT_SOURCE_DIR}/../../attribute_convert
  ${PROJECT_SOURCE_DIR}/../../singleton/
)
link_directories(
  ${PROJECT_SOURCE_DIR}/third_party/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/xproto/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/lib
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/lib
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/lib
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/lib
  ${PROJECT_SOURCE_DIR}/../../deps/uWS/lib
  ${PROJECT_SOURCE_DIR}/../../deps/protobuf/lib
  ${PROJECT_SOURCE_DIR}/../../deps/openssl/lib
)
# 头文件路径信息

if (${X3_MEDIA_CODEC})
set(BASE_LIBRARIES opencv_world pthread jsoncpp protobuf
    xstream_media_codec uWS ssl crypto z)
else ()
set(BASE_LIBRARIES opencv_world pthread jsoncpp protobuf uWS ssl crypto z)
endif()
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

#add_subdirectory(test)
#add_subdirectory(sample)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/configs/
    DESTINATION ${MY_OUTPUT_ROOT}/config/)
