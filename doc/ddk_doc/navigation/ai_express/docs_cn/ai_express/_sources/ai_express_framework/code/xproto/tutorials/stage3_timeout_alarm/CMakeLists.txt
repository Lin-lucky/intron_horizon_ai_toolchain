cmake_minimum_required(VERSION 2.8)

if (PLATFORM_ANDROID)
  set(BASE_LIBRARIES  xproto -llog libc++_static.a)
else()
  set(BASE_LIBRARIES  xproto rt pthread)
endif()

link_directories(
  ${LIB_AIEXRESS_DIR}/lib
)

set(SOURCE_FILES
        stage3_sample_plugin.cc
)

add_executable(stage3_timeout_alarm ${SOURCE_FILES})
target_link_libraries(stage3_timeout_alarm ${BASE_LIBRARIES})
