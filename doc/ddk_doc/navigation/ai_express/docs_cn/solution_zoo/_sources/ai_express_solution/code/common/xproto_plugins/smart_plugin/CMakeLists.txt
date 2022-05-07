cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
option(RELEASE_LIB "build version of release" ON)
# 项目名称
project(smart_plugin)

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
  include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/xproto/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio
  ${PROJECT_SOURCE_DIR}/../..
  ${PROJECT_SOURCE_DIR}/../../attribute_convert
  ${PROJECT_SOURCE_DIR}/../../singleton
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/include
  ${PROJECT_SOURCE_DIR}/../../deps/libjpeg-turbo/include
  ${PROJECT_SOURCE_DIR}/../../deps/protobuf/include
  ${PROJECT_SOURCE_DIR}/../../wrapper/media_codec/include
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/include
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/include
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/include
  ${PROJECT_SOURCE_DIR}/../message/include
)

link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${HOST_PACKAGE_DIR}/xproto/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/lib
  ${PROJECT_SOURCE_DIR}/../../deps/libjpeg-turbo/lib
  ${PROJECT_SOURCE_DIR}/../../deps/protobuf/lib
  ${PROJECT_SOURCE_DIR}/../../deps/zeroMQ/lib
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/lib
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/lib
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/lib
)

file(GLOB PROJECT_SRC
    "src/smart_plugin/*.cpp"
    "src/utils/*.cpp"
)
set(pb_src
  ${HOST_PACKAGE_DIR}/xproto/include/xproto/msg_type/protobuf/x3.pb.h
  ${HOST_PACKAGE_DIR}/xproto/include/xproto/msg_type/protobuf/x3.pb.cc
  ${PROJECT_SOURCE_DIR}/../message/include/protobuf/x3ware.pb.h
  ${PROJECT_SOURCE_DIR}/../message/include/protobuf/x3ware.pb.cc

) 
add_library(${PROJECT_NAME} SHARED ${PROJECT_SRC} ${pb_src})

set(CMAKE_CXX_STANDARD 11)
set(BASE_LIBRARIES hobotlog jsoncpp protobuf opencv_world
    turbojpeg pthread gtest xstream xproto xstream_media_codec z -ldl -lrt)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${X3_HAPI_LIB})

add_subdirectory(sample)

# make install才会安装
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
    DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
