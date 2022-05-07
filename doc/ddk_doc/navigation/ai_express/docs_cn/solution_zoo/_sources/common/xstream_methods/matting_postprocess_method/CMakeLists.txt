cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(matting_postprocess_method)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
  include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/bpu_predict/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../dnn_postprocess_method/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/opencv/include
  ../
)

link_directories(${CMAKE_SOURCE_DIR}/lib/)

set(SOURCE_FILES
        src/matting_postprocess_method.cc
)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

set(DYNAMIC_DEPS_LIBS bpu_predict cnn_intf hbrt_bernoulli_aarch64 cam vio)
target_link_libraries(${PROJECT_NAME} ${DYNAMIC_DEPS_LIBS})

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${PROJECT_SOURCE_DIR}/include/matting_postprocess_method/matting_postprocess_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/matting_postprocess_method)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
