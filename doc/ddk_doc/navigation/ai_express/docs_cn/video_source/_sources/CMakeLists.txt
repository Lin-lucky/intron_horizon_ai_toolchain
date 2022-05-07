cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)

if (NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

message("video source PARENT_BUILD:${PARENT_BUILD}")
option(PLATFORM_X3 "x3 platform" ON)
message("plugin_mode: ${PLUGIN_MODE}, release_mode: ${RELEASE_LIB}")
option(COVERAGE "Code Coverage " OFF)

# 项目名称
if (${PLUGIN_MODE})
  message("video source plugin project")
  project(video_source_plugin)
else ()
  message("video source project")
  project(video_source)
endif()

set(PLATFORM_LINUX TRUE)
if (${PLATFORM_MAC})
  add_definitions(-DHR_POSIX)
  add_definitions(-DHR_MAC)
elseif (${PLATFORM_LINUX})
  add_definitions(-DHR_POSIX)
  add_definitions(-DHR_LINUX)
  set(PLATFORM_NAME linux-x64)
elseif (${PLATFORM_ANDROID})
  set(ENABLE_NEON "true")
  add_definitions(-DENABLE_OMP)
  add_definitions(-DHR_POSIX)
  add_definitions(-DHR_ANDROID)
elseif (${PLATFORM_WIN})
  set(CMAKE_CXX_FLAGS_RELEASE "/MD")
  set(CMAKE_CXX_FLAGS_DEBUG "/MDd")
  add_definitions(-DHR_WIN)
  add_definitions(-D_MBCS)
  add_definitions(-DHOBOT_EXPORTS)
else ()
  message(FATAL_ERROR "invoke cmake with -DPLATFORM_MAC=TRUE|-DPLATFORM_LINUX=TRUE|-DPLATFORM_ANDROID=TRUE|-DPLATFORM_WIN=TRUE")
  return()
endif()

option(RELEASE_LIB "build version of release" ON)
message("config types: ${CMAKE_CONFIGURATION_TYPES}")

if (${PLATFORM_X3})
  # x3 soc
  add_definitions(-DX3)
  add_definitions(-DSUBSYS_NAME=IOT_VIO)
else ()
  message(FATAL_ERROR "invoke cmake with -DPLATFORM select")
  return()
endif()

if (${RELEASE_LIB})
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "BUILD TYPE:Release")
else ()
  set(CMAKE_BUILD_TYPE Debug)
  set(HOBOT_BUILD_DEBUG "TRUE")
  message(STATUS "BUILD TYPE:Debug")
endif ()

list(APPEND CMAKE_C_FLAGS "-Wall -Wno-unknown-pragmas -fPIC -fexceptions -Werror ")
list(APPEND CMAKE_C_FLAGS "-Wno-error=unused-function ")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  list(APPEND CMAKE_C_FLAGS "-O3")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC ")
else ()
  list(APPEND CMAKE_C_FLAGS "-g -O0 -fPIC ")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11 ")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
if (NOT ${PARENT_BUILD})
  set(OUTPUT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/output)
endif()
# set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT})
message("${PROJECT_NAME} MY_OUTPUT_ROOT: ${MY_OUTPUT_ROOT}")
# channge PARENT_BUILD value to build sub cmakelist
set(PARENT_BUILD ON)
# compile flags for coverage
if (COVERAGE)
    message("comiple video_source for coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O0 -g -fprofile-arcs -ftest-coverage")
endif ()

# 添加头文件路径
message("HOME value: $ENV{HOME}")
if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/hobotlog/include
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/include
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/gtest/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/libmm
)

link_directories(
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/hobotlog/lib
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/lib
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/opencv/lib
  ${PROJECT_SOURCE_DIR}/../third_party/aarch64/gtest/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia
  )

if (${PLUGIN_MODE})
include_directories(
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/xproto/include
  )
link_directories(
  ${HOST_PACKAGE_DIR}/xproto/lib
  ${HOST_PACKAGE_DIR}/xstream/lib
  )
endif()

# 源文件路径信息
file(GLOB PROJECT_BASE_SRC
  src/utils/executor.cc
  src/video_source/video_buffer/buffer_manager.cc
  src/video_source/decode/decode_manager.cc
  src/video_source/video_source.cc
  src/video_source/vin/vin_module.cc
  src/video_source/vps/vps_module.cc
  src/video_source/vps/vps_manager.cc
  src/video_source/vin/feedback/feedback_vin_module.cc
  src/video_source/vin/usb_cam/usb_cam_vin_module.cc
  src/video_source/vin/rtsp_client/rtsp_client_vin_module.cc
  )

if (${PLATFORM_X3})
  file(GLOB PROJECT_PLATFORM_SRC
    src/video_source/decode/x3/decode_module.cc
    src/video_source/vin/mipi_cam/x3/mipi_cam_vin_module_hapi.cc
    src/video_source/vin/mipi_cam/j3/mipi_cam_vin_module_vapi.cc
    src/video_source/vin/mipi_cam/x3/sensors/*.cc
    src/video_source/vps/x3/vps_module_hapi.cc
    src/video_source/vps/j3/vps_module_vapi.cc
    )
elseif (${PLATFORM_J5})
endif()

if (${PLUGIN_MODE})
  file(GLOB PROJECT_PLUGIN_SRC
    src/video_source_plugin/video_source_config.cc
    src/video_source_plugin/video_source_message.cc
    src/video_source_plugin/video_source_plugin.cc
    src/video_source_plugin/video_source_produce.cc
    )
endif()

set(SOURCE_FILES ${PROJECT_BASE_SRC} ${PROJECT_PLATFORM_SRC} ${PROJECT_PLUGIN_SRC})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# link library
set(BASE_LIBRARIES hobotlog.a jsoncpp.a dl m pthread rt)
if (${PLATFORM_X3})
set(X3_HAPI_LIB vio cam uvc isp iar isp_algo tinyalsa avformat avcodec avutil
  multimedia hbmedia z)
endif()
if (${PLUGIN_MODE})
  set(PLUGIN_LIB xproto xstream)
endif()
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${X3_HAPI_LIB} ${PLUGIN_LIB})

add_subdirectory(src/video_source/vin/mipi_cam/x3/sensors/plugin)

# sample test
if (${PLUGIN_MODE})
  add_subdirectory(sample/video_source_plugin)
else()
  add_subdirectory(sample/video_source)
  # gtest
  add_subdirectory(test/video_source)
endif()

set(SENSOR_LIB_DIR
  ${PROJECT_SOURCE_DIR}/src/video_source/vin/mipi_cam/x3/sensors/lib)
set(FFMPEG_LIB_DIR
  ${PROJECT_SOURCE_DIR}/src/video_source/vin/rtsp_client/lib)

if (${PLUGIN_MODE})
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/video_source_plugin
    DESTINATION ${MY_OUTPUT_ROOT}/include)
else()
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/video_source
    DESTINATION ${MY_OUTPUT_ROOT}/include)
endif()
install(FILES ${PROJECT_SOURCE_DIR}/src/video_source/version.h
    DESTINATION ${MY_OUTPUT_ROOT}/include)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/configs/
    DESTINATION ${MY_OUTPUT_ROOT}/configs)

install(FILES ${SENSOR_LIB_DIR}/../README.md
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md
    DESTINATION ${MY_OUTPUT_ROOT})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/image
    DESTINATION ${MY_OUTPUT_ROOT})
install(DIRECTORY ${SENSOR_LIB_DIR}/f37/
    DESTINATION ${MY_OUTPUT_ROOT}/lib/sensor)
install(DIRECTORY ${SENSOR_LIB_DIR}/imx586/
    DESTINATION ${MY_OUTPUT_ROOT}/lib/sensor)
install(DIRECTORY ${FFMPEG_LIB_DIR}/
    DESTINATION ${MY_OUTPUT_ROOT}/lib/ffmpeg)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)

