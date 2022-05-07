cmake_minimum_required(VERSION 2.8)

# 设置头文件搜索路径
include_directories(./)

set(BASE_LIBRARIES xstream hobotlog pthread)

set(SOURCE_FILES
  method/method_factory.cc)

add_executable(benchmark_main
  benchmark_main.cc
  ${SOURCE_FILES})
target_link_libraries(benchmark_main ${BASE_LIBRARIES})
