cmake_minimum_required(VERSION 2.8)

include_directories(./)

set(BASE_LIBRARIES xstream jsoncpp pthread rt)

set(SOURCE_FILES
    method/bbox.h
    method/bbox_filter.h
    method/bbox_filter.cc
    method/method_factory.cc
    method/filter_param.h)

add_executable(stage3_update_parameter_example
               update_param_main.cc
               ${SOURCE_FILES})

target_link_libraries(stage3_update_parameter_example ${BASE_LIBRARIES})