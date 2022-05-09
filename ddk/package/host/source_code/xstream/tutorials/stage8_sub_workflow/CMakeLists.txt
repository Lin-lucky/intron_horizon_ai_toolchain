cmake_minimum_required(VERSION 2.8)
# 项目名称
#project(stage8)

# 设置头文件搜索路径
include_directories(./)

set(BASE_LIBRARIES xstream pthread rt)

set(SOURCE_FILES
  method/bbox.h
  method/gray_image.h
  method/bbox_filter.h
  method/bbox_filter.cc
  method/method_factory.cc
  method/faster_detect.h
  method/faster_detect.cc)

add_executable(stage8_sub_workflow
  sub_workflow_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage8_sub_workflow ${BASE_LIBRARIES})

