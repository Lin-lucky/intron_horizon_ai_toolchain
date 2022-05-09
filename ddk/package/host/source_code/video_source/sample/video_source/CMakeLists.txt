cmake_minimum_required(VERSION 2.8)

# 项目名称
project(video_source_sample)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

set(SOURCE_FILES sample.cpp)

set(BASE_LIBRARIES video_source)
# set(SCRIPT_FILES ${PROJECT_SOURCE_DIR}/run.sh)

# 添加依赖
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/bin)
# install(FILES ${SCRIPT_FILES} DESTINATION ${MY_OUTPUT_ROOT}/)
