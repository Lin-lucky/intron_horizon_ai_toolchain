cmake_minimum_required(VERSION 2.8)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
)

set(BASE_LIBRARIES xstream pthread dl rt)

add_executable(stage7_node_output
  ${CMAKE_CURRENT_LIST_DIR}/include/method/bbox.h
  ${CMAKE_CURRENT_LIST_DIR}/include/method/bbox_filter_a.h
  ${CMAKE_CURRENT_LIST_DIR}/include/method/bbox_filter_a.cc
  ${CMAKE_CURRENT_LIST_DIR}/include/method/bbox_filter_b.h
  ${CMAKE_CURRENT_LIST_DIR}/include/method/bbox_filter_b.cc
  ${CMAKE_CURRENT_LIST_DIR}/include/method/callback.h
  ${CMAKE_CURRENT_LIST_DIR}/include/method/method_factory.cc
  src/main.cc
  
)
target_link_libraries(stage7_node_output 
  PUBLIC ${BASE_LIBRARIES}
)
