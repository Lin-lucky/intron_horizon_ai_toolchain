cmake_minimum_required(VERSION 2.8)

option(INDEPENDENT_BUILD "is build independent" OFF)
if(${INDEPENDENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
  if(DEFINED ENV{HORIZON_LIB_PATH})
    set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
  else()
    set(HOST_PACKAGE_DIR $ENV{HOME}/.horizon/ddk/xj3_aarch64)
  endif()
  include_directories(
    # xstream
    ${HOST_PACKAGE_DIR}/xstream/include/
)
link_directories(
  # xstream
  ${HOST_PACKAGE_DIR}/xstream/lib/
)
endif()

# 设置头文件搜索路径
include_directories(./)

set(BASE_LIBRARIES xstream pthread rt)

set(SOURCE_FILES
  method/bbox.h
  method/bbox_filter.h
  method/method_factory.cc)

add_executable(stage1_hello_world
  bbox_filter_main.cc
  ${SOURCE_FILES})
target_link_libraries(stage1_hello_world ${BASE_LIBRARIES})

if(${INDEPENDENT_BUILD})
  set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output)
  install(FILES ${CMAKE_SOURCE_DIR}/config/bbox_workflow.json
        DESTINATION ${OUTPUT_ROOT})
  install(FILES ${HOST_PACKAGE_DIR}/xstream/lib/libxstream.so
        DESTINATION ${OUTPUT_ROOT})
  install(TARGETS stage1_hello_world
        DESTINATION ${OUTPUT_ROOT})
endif()
