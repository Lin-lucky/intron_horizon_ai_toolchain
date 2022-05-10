cmake_minimum_required(VERSION 2.8)

include_directories(
    include
)

set(XSTREAM_LB -Wl,--whole-archive xstream -Wl,--no-whole-archive)

add_executable(${target_name}_example
    main.cc
    ../test/src/method_factory.cc
    test_anti_spf_calculate.cc)

target_link_libraries(${target_name}_example
    ${target_name} hobotlog jsoncpp pthread dl rt
)
