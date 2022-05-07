cmake_minimum_required(VERSION 2.8)

include_directories(
  .
)
set(SOURCE_FILES method_factory.cc method/multisource_method.cc)

SET(BASE_LIBRARIES xstream jsoncpp rt pthread)
add_executable(stage4_multisource multisource_main.cc ${SOURCE_FILES})
target_link_libraries(stage4_multisource ${BASE_LIBRARIES})

