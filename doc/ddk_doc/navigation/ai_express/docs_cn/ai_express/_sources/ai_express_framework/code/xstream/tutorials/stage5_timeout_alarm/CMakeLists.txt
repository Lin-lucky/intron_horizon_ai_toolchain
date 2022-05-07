cmake_minimum_required(VERSION 2.8)

# 设置头文件搜索路径
include_directories(./)

set(BASE_LIBRARIES xstream  pthread rt)

set(SOURCE_FILES
  method/alarm_method.cpp
  method_factory.cc)

add_executable(stage5_timeout_alarm_sync_example
  timeout_alarm_sync_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage5_timeout_alarm_sync_example ${BASE_LIBRARIES})