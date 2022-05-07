cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
option(COVERAGE "Code Coverage " OFF)
message("tool chain:${CMAKE_CXX_COMPILER}")
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURDIR ${CMAKE_CURRENT_SOURCE_DIR})
message("${CMAKE_CURRENT_SOURCE_DIR}")
# 项目名称
project(uvc_server)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

if (COVERAGE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O0 -g -fprofile-arcs -ftest-coverage")
endif ()

# 添加头文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/live555/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/opencv/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/include
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include 
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio/
)

link_directories(${HOST_PACKAGE_DIR}/xstream/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/live555/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/opencv/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
)

file(GLOB PROJECT_INC
    "include/*.h"
    "include/uvc_server/*.h"
)
file(GLOB PROJECT_SRC 
    "src/*.cc" 
    "src/uvc_server/*.cc"
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})

set(BASE_LIBRARIES pthread  jsoncpp hobotlog
)

set(UVC_LIB guvc)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}
  PUBLIC ${BASE_LIBRARIES}
  PUBLIC ${UVC_LIB} 
)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/uvc_server)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

add_subdirectory(sample/01_transfer_nv12)

add_subdirectory(test)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/uvc_server/
    DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/test/data/
    DESTINATION ${MY_OUTPUT_ROOT}/data/)
install(PROGRAMS ${PROJECT_SOURCE_DIR}/test/run_gtest.sh
    DESTINATION ${MY_OUTPUT_ROOT})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
