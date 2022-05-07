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
    # xproto
    ${HOST_PACKAGE_DIR}/xproto/include/
)
link_directories(
  # xproto
  ${HOST_PACKAGE_DIR}/xproto/lib/
)
endif()

if (PLATFORM_ANDROID)
  set(BASE_LIBRARIES  xproto -llog libc++_static.a)
else()
  set(BASE_LIBRARIES  xproto rt pthread)
endif()

link_directories(
  ${LIB_AIEXRESS_DIR}/lib
)

set(SOURCE_MASTER_FILES
        stage4_master_plugin.cc
)

set(SOURCE_SLAVE_FILES
        stage4_slave_plugin.cc
)

add_executable(stage4_ipc_subscriber_master ${SOURCE_MASTER_FILES})
target_link_libraries(stage4_ipc_subscriber_master ${BASE_LIBRARIES})

add_executable(stage4_ipc_subscriber_slave ${SOURCE_SLAVE_FILES})
target_link_libraries(stage4_ipc_subscriber_slave ${BASE_LIBRARIES})

if(${INDEPENDENT_BUILD})
  set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output)
  install(FILES ${HOST_PACKAGE_DIR}/xproto/lib/libxproto.so
        DESTINATION ${OUTPUT_ROOT})
  install(TARGETS stage4_ipc_subscriber_master
        DESTINATION ${OUTPUT_ROOT})
  install(TARGETS stage4_ipc_subscriber_slave
        DESTINATION ${OUTPUT_ROOT})
endif()