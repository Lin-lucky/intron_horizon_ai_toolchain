cmake_minimum_required(VERSION 2.8)

project(uvc_server_gtest)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/gtest/include
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/libmm
    ${HOST_PACKAGE_DIR}/uvc_server/include/
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/gtest/lib)

set(SOURCE_FILES
        gtest_main.cc
        uvc_server_gtest.cc
        src/transfer_format.cc
        )

set(BASE_LIBRARIES uvc_server gtest)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

set(FFMPEG_LIBRARIES avformat avutil avcodec)

target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${FFMPEG_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/bin)