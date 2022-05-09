cmake_minimum_required(VERSION 2.8)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# x86 architecture
if (${X86_ARCH})
  # centos release
  if (${X86_CENTOS})
    include_directories(
      ${XSTREAM_CURRENT_DIR}/framework/third_party/x86/centos/gtest/include
    )
    link_directories(
      ${XSTREAM_CURRENT_DIR}/framework/third_party/x86/centos/hobotlog/lib
      ${XSTREAM_CURRENT_DIR}/framework/third_party/x86/centos/gtest/lib
    )
  elseif(${ARM_ANDROID})
    include_directories(
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/gtest/include
    )
    link_directories(
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/hobotlog/lib
      ${XPROTO_CURRENT_DIR}/xproto/third_party/arm/android/gtest/lib
    )
  else ()
    include_directories(
      ${XSTREAM_CURRENT_DIR}/framework/third_party/x86/ubuntu/gtest/include
    )
    link_directories(
      ${XSTREAM_CURRENT_DIR}/framework/third_party/x86/ubuntu/hobotlog/lib
      ${XSTREAM_CURRENT_DIR}/framework/third_party/x86/ubuntu/gtest/lib
    )
  endif()
# arm architecture  
else ()
  include_directories(
    ${RELEASE_ROOT_DIR}/deps/gtest/include
  )

  link_directories(
    ${RELEASE_ROOT_DIR}/deps/gtest/lib
    ${RELEASE_ROOT_DIR}/deps/hobotlog/lib
  )
endif()

# test case的所有cpp文件
set(SOURCE_FILES
  gtest_main.cc
  test_api.cc
  test_xplugin.cc
)
# 添加依赖
## base deps
if (PLATFORM_WIN)
  set(BASE_LIBRARIES gtest.lib hobotlog.lib)
else ()
  if (PLATFORM_ANDROID)
    set(BASE_DEPS -ldl -lm -llog libc++_static.a)
  else()
    set(BASE_DEPS -ldl -lm -lpthread -lrt)
  endif()
  set(BASE_LIBRARIES ${BASE_LIBRARIES} libgtest.a libhobotlog.a)
  set(BASE_LIBRARIES ${BASE_LIBRARIES} ${BASE_DEPS})
endif ()

# hobot_util.cmake已经设置了在build/bin中生成可执行文件，默认应该在test目录里生成
add_executable(xproto_unit_test ${SOURCE_FILES})
target_link_libraries(xproto_unit_test
  PUBLIC ${PROJECT_NAME} ${BASE_LIBRARIES}
)
