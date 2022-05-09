cmake_minimum_required(VERSION 2.8)

# project name
project(xproto)

option(PARENT_BUILD "is build from parent" OFF)
option(INSTALL_TUTORIALS "is install tutorials " OFF)
option(BUILD_ALL_ARCH "is build all architecture " OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT ${PARENT_BUILD})
  get_filename_component(XPROTO_CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
  #message("xproto current dir init:${XPROTO_CURRENT_DIR}")
  set(RELEASE_ROOT_DIR)
  function(get_parent_dir arg dir n)
    foreach(i RANGE ${ARGV2})
      get_filename_component(dir ${dir} DIRECTORY)
      #message("i:${i} dir:${dir}")
    endforeach()
    set(RELEASE_ROOT_DIR ${dir} PARENT_SCOPE)
  endfunction()

  get_parent_dir(XPROTO_CURRENT_DIR ${XPROTO_CURRENT_DIR} 2)
  option(X86_ARCH "x86 architecture" OFF)
  option(X86_CENTOS "centos release" OFF)
  option(ARM_ANDROID "3399 android" OFF)
endif()

option(RELEASE_LIB "build version of release" ON)
option(BUILD_SHARED_LIBS "build shared libs" ON)
if (${BUILD_SHARED_LIBS} AND ${RELEASE_LIB})
set(CMAKE_SHARED_LINKER_FLAGS " -Wl,--version-script=${PROJECT_SOURCE_DIR}/export_lib.lst")
endif()

if (NOT ${PARENT_BUILD})
  if (${RELEASE_LIB})
    set(CMAKE_BUILD_TYPE Release)
    message(STATUS "BUILD TYPE:Release")
  else ()
    set(CMAKE_BUILD_TYPE Debug)
    set(HOBOT_BUILD_DEBUG "TRUE")
    message(STATUS "BUILD TYPE:Debug")
  endif ()

  list(APPEND CMAKE_C_FLAGS "-Wall -Wno-unknown-pragmas -fPIC -fexceptions -Werror ")
  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    list(APPEND CMAKE_C_FLAGS "-O3")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC ")
  else ()
    list(APPEND CMAKE_C_FLAGS "-g -O0 -fPIC ")
  endif ()

  # x86 architecture
  if (${X86_ARCH})
    # centos release
    if (${X86_CENTOS})
      message("tool chain:${CMAKE_CXX_COMPILER}")
      add_definitions(-std=c++11)
      add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
      include_directories(
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/hobotlog/include
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/gtest/include
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/protobuf/include
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/zeroMQ/include
      )
      link_directories(
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/hobotlog/lib
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/gtest/lib
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/protobuf/lib
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/centos/zeroMQ/lib
      )
    else ()
      message("tool chain:${CMAKE_CXX_COMPILER}")
      add_definitions(-std=c++11)
      include_directories(
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/gtest/include
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/hobotlog/include
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/protobuf/include
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/zeroMQ/include
      )
      link_directories(
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/gtest/lib
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/hobotlog/lib
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/protobuf/lib
        ${XPROTO_CURRENT_DIR}/xproto/third_party/x86/ubuntu/gcc_${CMAKE_CXX_COMPILER_VERSION}/zeroMQ/lib
      )
    endif ()
  # arm architecture
  elseif(${ARM_ANDROID})
    SET(PLATFORM_ANDROID TRUE)
    message(${PLATFORM_ANDROID})
    set(CMAKE_CXX_COMPILER /opt/android-standalone-toolchain/bin/arm-linux-androideabi-g++)
    message("tool chain:${CMAKE_CXX_COMPILER}")
    include_directories(
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/hobotlog/include
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/gtest/include
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/protobuf/include
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/zeroMQ/include
    )
    link_directories(
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/hobotlog/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/gtest/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/protobuf/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/zeroMQ/lib
    )
  else ()
    set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
    message("tool chain:${CMAKE_CXX_COMPILER}")
    include_directories(
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/hobotlog/include
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/gtest/include
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/protobuf/include
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/zeroMQ/include
    )
    link_directories(
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/hobotlog/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/gtest/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/protobuf/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/aarch64/zeroMQ/lib
    )
  endif ()

  # build separately install directory
  set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif ()

# add header files path
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src/include
)

# header files
set(PROJECT_INC
  "include/xproto/xproto_world.h"
  "src/include/xproto/msg_manager.h"
  "src/include/xproto/threadpool.h"
  "include/xproto/msg_type/protobuf/x3.pb.h"
  "include/xproto/msg_type/protobuf/x3.pb.cc"
  "include/xproto/msg_type/control_message.h"
  "include/xproto/msg_type/statistics_message.h"
  "include/xproto/msg_type/smart_legible_message.h"
)

# source files
set(PROJECT_SRC
  "src/message/msg_registry.cc"
  "src/plugin/xplugin.cc"
  "src/plugin/xpluginasync.cc"
  "src/threads/threadpool.cc"
  "src/utils/time_helper.cc"
  "src/session/xsession_inner.cc"
  "src/session/xsession.cc"
  "src/session/zmq_manager.cc"
  "src/msg_type/control_message.cc"
  "src/msg_type/smart_legible_message.cc"
  "src/msg_type/statistics_message.cc"
)

set(SOURCE_FILES
  ${PROJECT_SRC}
  ${PROJECT_INC}
)

option(BUILD_SHARED_LIBS "build shared libs" ON)
add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
  PUBLIC hobotlog libprotobuf.a libzmq.a)

add_subdirectory(test)
add_subdirectory(tutorials/stage1_hello_world)
add_subdirectory(tutorials/stage2_max_queue_size)
add_subdirectory(tutorials/stage3_timeout_alarm)
add_subdirectory(tutorials/stage4_ipc_subscriber)

# make install
# set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/output)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/xproto)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/plugin/xplugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/plugin/xpluginasync.h
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/plugin
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/message/flowmsg.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/message/msg_registry.h
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/message/
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/version.h
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/session/xsession_info.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/session/xsession.h
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/session/
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/msg_type/control_message.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/msg_type/smart_legible_message.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/msg_type/statistics_message.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/msg_type/vio_message.h
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/msg_type/
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xstream
  DESTINATION ${MY_OUTPUT_ROOT}/include
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/msg_type/protobuf
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/msg_type/
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/xproto/xproto_world.h
  DESTINATION ${MY_OUTPUT_ROOT}/include/xproto/
)
if (${BUILD_ALL_ARCH})
  if(${X86_ARCH})
    # centos
    if (${X86_CENTOS})
      # .so
      if (${BUILD_SHARED_LIBS})
        # release
        if (${RELEASE_LIB})
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/centos/shared_lib/
          )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/centos/shared_lib/debug
          )
        endif()
      # .a
      else()
        # release
        if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/centos/static_lib/
        )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/centos/static_lib/debug
        )
        endif()
      endif()
    # ubuntu
    else ()
      # .so
      if (${BUILD_SHARED_LIBS})
        # release
        if (${RELEASE_LIB})
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/shared_lib/
          )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/shared_lib/debug
          )
        endif()
      # .a
      else()
        # release
        if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/static_lib/
        )
        # debug
        else ()
          install(TARGETS ${PROJECT_NAME}
            DESTINATION ${MY_OUTPUT_ROOT}/ubuntu/static_lib/debug
        )
        endif()
      endif()
    endif()
  # android_3399
  elseif(${ARM_ANDROID})
    # .so
    if (${BUILD_SHARED_LIBS})
      # release
      if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/android_3399/shared_lib/
        )
      # debug
      else ()
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/android_3399/shared_lib/debug
        )
      endif()
    # .a
    else()
      # release
      if (${RELEASE_LIB})
      install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/android_3399/static_lib/
      )
      # debug
      else ()
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/android_3399/static_lib/debug
      )
      endif()
    endif()
  # xj3j5
  else()
    # .so
    if (${BUILD_SHARED_LIBS})
      # release
      if (${RELEASE_LIB})
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/shared_lib/
        )
      # debug
      else ()
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/shared_lib/debug
        )
      endif()
    # .a
    else()
      # release
      if (${RELEASE_LIB})
      install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/static_lib/
      )
      # debug
      else ()
        install(TARGETS ${PROJECT_NAME}
          DESTINATION ${MY_OUTPUT_ROOT}/xj3j5/static_lib/debug
      )
      endif()
    endif()
  endif()
else()
  install(TARGETS ${PROJECT_NAME}
	  DESTINATION ${MY_OUTPUT_ROOT}/lib
  )
endif()

if (${INSTALL_TUTORIALS})
  install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md
    DESTINATION ${MY_OUTPUT_ROOT}/
  )

  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/document
    DESTINATION ${MY_OUTPUT_ROOT}/
  )

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tutorials
    DESTINATION ${MY_OUTPUT_ROOT}/
  )

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    DESTINATION ${MY_OUTPUT_ROOT}/tutorials
  )
endif()
