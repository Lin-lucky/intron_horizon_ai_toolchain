cmake_minimum_required(VERSION 2.8)
# 项目名称
#project(stage9)

# 设置头文件搜索路径
include_directories(./)

set(BASE_LIBRARIES xstream pthread rt)

set(SOURCE_FILES
  method/bbox.h
  method/bbox_filter.h
  method/bbox_filter.cc
  method/method_factory.cc)

add_executable(stage9_disable_method_example
  disable_method_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage9_disable_method_example ${BASE_LIBRARIES})

add_executable(stage9_use_predefined_method_example
  use_predefined_method_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage9_use_predefined_method_example ${BASE_LIBRARIES})

add_executable(stage9_pass_through_method_example
  pass_through_method_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage9_pass_through_method_example ${BASE_LIBRARIES})

add_executable(stage9_best_effort_pass_through_method_example
  best_effort_pass_through_method_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage9_best_effort_pass_through_method_example ${BASE_LIBRARIES})

add_executable(stage9_best_effort_pass_through_method_example_2
  best_effort_pass_through_method_main_2.cc
  ${SOURCE_FILES})
target_link_libraries(stage9_best_effort_pass_through_method_example_2 ${BASE_LIBRARIES})