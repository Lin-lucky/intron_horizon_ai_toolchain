cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(lowpass_filter_method)

STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}) 

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${HOST_PACKAGE_DIR}/xstream/include
)

set(SOURCE_FILES
        src/lowpass_filter_method/lowpass_filter_method.cpp
        )

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

set(BASE_LIBRARIES xstream hobotlog jsoncpp opencv_world)

target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
        DESTINATION ${MY_OUTPUT_ROOT})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
