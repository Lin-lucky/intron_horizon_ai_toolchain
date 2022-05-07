cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

# 项目名称
project(media_codec_test)
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)
set(SOURCE_FILES
        gtest_main.cc
        media_codec_test.cpp
        )

set(BASE_LIBRARIES pthread dl rt)
set(X3_HAPI_LIB isp iar isp_algo tinyalsa multimedia z avformat hbmedia)
set(DepsLib xstream_media_codec hobotlog gtest)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
    ${DepsLib}
    ${X3_HAPI_LIB}
    ${BASE_LIBRARIES})
