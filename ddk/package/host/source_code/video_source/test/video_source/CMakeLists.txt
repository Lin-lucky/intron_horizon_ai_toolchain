cmake_minimum_required(VERSION 2.8)

# 项目名称
project(video_source_gtest)
include_directories(
    ${PROJECT_SOURCE_DIR}
)
set(SOURCE_FILES
        gtest_main.cc
        video_source_gtest.cc
        )

set(BASE_LIBRARIES video_source gtest)
# set(SCRIPT_FILES ${PROJECT_SOURCE_DIR}/run_gtest.sh)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/bin)
# install(FILES ${SCRIPT_FILES} DESTINATION ${MY_OUTPUT_ROOT}/)
