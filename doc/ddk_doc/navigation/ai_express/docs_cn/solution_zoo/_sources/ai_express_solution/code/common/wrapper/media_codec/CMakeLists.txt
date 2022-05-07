cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

string(REGEX REPLACE ".*/\(.*\)" "\\1" CURDIR ${CMAKE_CURRENT_SOURCE_DIR})
message("${CMAKE_CURRENT_SOURCE_DIR}")
message("tool chain:${CMAKE_CXX_COMPILER}")
# 项目名称
project(xstream_media_codec)

STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}) 

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

# 添加头文件路径
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio/
)

link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
                     )

file(GLOB PROJECT_INC
    "include/media_codec/*.h"
    "include/utils/*.h"
)

file(GLOB PROJECT_SRC
    "src/media_codec/*.cpp"
    "src/utils/*.cpp"
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})

set(X3_HAPI_LIB isp iar isp_algo tinyalsa multimedia z avformat hbmedia)
option(BUILD_SHARED_LIBS "build shared libs" ON)
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${X3_HAPI_LIB})
add_subdirectory(test)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/alsa_device)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include/)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
