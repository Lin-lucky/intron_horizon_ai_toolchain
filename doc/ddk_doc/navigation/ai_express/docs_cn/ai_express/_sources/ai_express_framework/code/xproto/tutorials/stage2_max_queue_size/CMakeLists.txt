cmake_minimum_required(VERSION 2.8)

# 设置头文件搜索路径
include_directories(include)

if (PLATFORM_ANDROID)
  set(BASE_LIBRARIES  xproto -llog libc++_static.a)
else()
  set(BASE_LIBRARIES  xproto rt pthread)
endif()

link_directories(
  ${LIB_AIEXRESS_DIR}/lib
)

set(SOURCE_FILES
        stage2_sample_plugin.cc
)

add_executable(stage2_max_queue_size ${SOURCE_FILES})
target_link_libraries(stage2_max_queue_size ${BASE_LIBRARIES})
