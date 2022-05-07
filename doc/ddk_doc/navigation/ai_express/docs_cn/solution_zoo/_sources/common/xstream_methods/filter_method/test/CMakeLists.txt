include_directories(
  include
  ${SOLUTION_ZOO_COMMON_DIR}/deps/gtest/include/
)

set(SOURCE_FILES
  gtest_main.cc
  test_main.cc
)

add_executable(filter_test ${SOURCE_FILES})

if (${PLATFORM_WIN})
     set(CMAKE_CXX_FLAGS_RELEASE "/MT")
     set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
     target_link_libraries(filter_test debug gtestd)
     target_link_libraries(filter_test optimized gtest)
else()
     target_link_libraries(filter_test  ${PROJECT_NAME} jsoncpp hobotlog xstream gtest pthread dl)
endif()
