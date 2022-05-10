cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "build subdirectory from here" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
# 项目名称
project(01_transfer_nv12)
set(CMAKE_CXX_STANDARD 11)
STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
# 添加头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(NOT ${PARENT_BUILD})
include_directories(
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/uvc_server/include
)
link_directories(
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/uvc_server/lib
)
endif()

file(GLOB PROJECT_SRC 
    "src/*.cc"
    "main.cc"
)

set(SOURCE_FILES ${PROJECT_SRC})

set(BASE_LIBRARIES pthread uvc_server)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

