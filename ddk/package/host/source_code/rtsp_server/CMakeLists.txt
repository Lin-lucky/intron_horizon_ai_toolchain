cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
option(COVERAGE "Code Coverage " OFF)
option(RELEASE_LIB "build release version" ON)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()

if (${RELEASE_LIB})
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "BUILD TYPE:Release")
else ()
  set(CMAKE_BUILD_TYPE Debug)
  set(HOBOT_BUILD_DEBUG "TRUE")
  message(STATUS "BUILD TYPE:Debug")
endif ()

message("tool chain:${CMAKE_CXX_COMPILER}")
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURDIR ${CMAKE_CURRENT_SOURCE_DIR})
message("${CMAKE_CURRENT_SOURCE_DIR}")
# project name
project(rtsp_server)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

if (COVERAGE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O0 -g -fprofile-arcs -ftest-coverage")
endif ()

# add include route
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rtsp_server
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/live555/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/opencv/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/include
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/libmm
)
# add libraries route
link_directories(${HOST_PACKAGE_DIR}/xstream/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/live555/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/opencv/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia
)

file(GLOB PROJECT_INC
    "include/*.h"
    "include/rtsp_server/*.h"
)
file(GLOB PROJECT_SRC 
    "src/*.cc" 
    "src/rtsp_server/*.cc"
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})

set(BASE_LIBRARIES pthread  jsoncpp hobotlog
)

set(LIVE_LIB liveMedia BasicUsageEnvironment groupsock UsageEnvironment)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} 
  PUBLIC ${BASE_LIBRARIES}
  PUBLIC ${LIVE_LIB} 
)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/rtsp_server)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

add_subdirectory(example)
add_subdirectory(test)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/rtsp_server/
    DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)
# install configs, some scripts will depends this
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/configs/
    DESTINATION ${MY_OUTPUT_ROOT}/configs/)
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/data/
    DESTINATION ${MY_OUTPUT_ROOT}/data)
