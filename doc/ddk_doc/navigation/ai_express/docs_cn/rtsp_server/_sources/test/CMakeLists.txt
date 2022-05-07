cmake_minimum_required(VERSION 2.8)

project(rtsp_server_gtest)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/gtest/include
    ${PROJECT_SOURCE_DIR}
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/gtest/lib)

set(SOURCE_FILES
        gtest_main.cc
        rtsp_server_gtest.cc
        )

set(BASE_LIBRARIES rtsp_server gtest)

set(FFMPEG_LIBRARIES avformat avutil avcodec)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${FFMPEG_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/bin)
