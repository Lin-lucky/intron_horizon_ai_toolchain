cmake_minimum_required(VERSION 2.8)

project(rtsp_server_example)

file(GLOB PROJECT_INC
    "include/*.h"
)
file(GLOB PROJECT_SRC "*.cc" "src/*.cc")

string(REGEX REPLACE ".*/\(.*\)" "\\1" CURDIR ${CMAKE_CURRENT_SOURCE_DIR})

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/rtsp_server
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/libmm
)

link_directories(
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib   
)

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})
set(SCRIPT_FILES ${PROJECT_SOURCE_DIR}/run.sh)

set(BASE_LIBRARIES rtsp_server)
set(FFMPEG_LIBRARIES avformat avutil avcodec)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${FFMPEG_LIBRARIES} rt)

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/bin)
install(FILES ${SCRIPT_FILES} DESTINATION ${MY_OUTPUT_ROOT}/)
