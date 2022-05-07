cmake_minimum_required(VERSION 2.8)

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/include
)

set(SOURCE_FILES
        ${CMAKE_CURRENT_LIST_DIR}/method_factory.cc
        )

add_executable(merge_unit_test1 ${SOURCE_FILES} test_main1.cc)
target_link_libraries(merge_unit_test1 merge_method ${BASE_LIBRARIES} gtest pthread dl rt)
add_executable(merge_unit_test2 ${SOURCE_FILES} test_main2.cc)
target_link_libraries(merge_unit_test2 merge_method ${BASE_LIBRARIES} gtest pthread dl rt)

