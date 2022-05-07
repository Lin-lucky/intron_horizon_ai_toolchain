cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(vehicle_plate_match_method)

message("${PROJECT_SOURCE_DIR}")
set(target_name vehicle_plate_match_method)
if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
include_directories(
    include
    ${HOST_PACKAGE_DIR}/xstream/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/include
)

link_directories(
    ${HOST_PACKAGE_DIR}/xstream/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/lib
)

file(GLOB SOURCE_FILES
    "src/*.cc"
    "src/match_utility/*.cpp"
    "src/match_strategy/*.cpp"
)

add_library(${target_name} STATIC ${SOURCE_FILES})
set(BASE_LIBRARIES xstream jsoncpp hobotlog pthread)
target_link_libraries(${target_name} ${BASE_LIBRARIES})

add_subdirectory(test)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${target_name}/)
install(FILES ${PROJECT_SOURCE_DIR}/include/${target_name}/vehicle_plate_match_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/${target_name})
install(TARGETS ${target_name}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
