cmake_minimum_required(VERSION 2.8)

include_directories(
        include
)

set(SOURCE_FILES
        ${CMAKE_CURRENT_LIST_DIR}/method_factory.cc
        ${CMAKE_CURRENT_LIST_DIR}/gtest_main.cc
        )

add_executable(gtest_head_face ${SOURCE_FILES} gtest_head_face.cc)
target_link_libraries(gtest_head_face merge_method ${BASE_LIBRARIES} gtest pthread dl rt)
