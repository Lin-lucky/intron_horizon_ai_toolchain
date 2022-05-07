cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
option(RELEASE_LIB "build version of release" ON)

project(CNNMethod)

message("${PROJECT_SOURCE_DIR}")
option(USE_BGR2NV12 "select BGR to convert directly to nv12" OFF)
add_definitions(-DSEARCH_PERFORMANCE)
message("USE_BGR2NV12 is ${USE_BGR2NV12}")
if(USE_BGR2NV12)
    add_definitions(-DUSE_BGR2NV12)
    message("select BGR to convert directly to nv12")
endif(USE_BGR2NV12)


option(COVERAGE_TEST "open coverage build flag" OFF)
message("COVERAGE_TEST is ${COVERAGE_TEST}")
if (${COVERAGE_TEST})
  list(APPEND CMAKE_C_FLAGS " --coverage")
endif()
if ((NOT (${RELEASE_LIB})) OR (${COVERAGE_TEST}))
  list(APPEND CMAKE_C_FLAGS " -g -Og")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11 ")


if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
include_directories(
  include
  ${HOST_PACKAGE_DIR}/bpu_predict/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${HOST_PACKAGE_DIR}/image_utils/include
  ${HOST_PACKAGE_DIR}/xstream/include
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include
  ${PROJECT_SOURCE_DIR}/../..
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/include
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/include
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/include
  ${PROJECT_SOURCE_DIR}/../../deps
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/include
)

set(SOURCE_FILES
    src/cnn_method.cc
    src/cnn_const.cc
    src/util/model_info.cc
    src/util/align_face.cc
    src/predictor/predictor.cc
    src/predictor/rect_input_predictor.cc
    src/predictor/img_input_predictor.cc
    src/predictor/lmk_input_predictor.cc
    src/predictor/vehicle_img_input_predictor.cc
    src/predictor/lmk_sequence_input_predictor.cc
    src/predictor/vid_input_predictor.cc
    src/post_predictor/post_predictor.cc
    src/post_predictor/age_gender_post_predictor.cc
    src/post_predictor/binary_classify_post_predictor.cc
    src/post_predictor/antispf_post_predictor.cc
    src/post_predictor/faceid_post_predictor.cc
    src/post_predictor/lmk_pose_post_predictor.cc
    src/post_predictor/platenum_post_predictor.cc
    src/post_predictor/vehicle_color_post_predictor.cc
    src/post_predictor/vehicle_type_post_predictor.cc
    src/post_predictor/classify_post_predictor.cc
    src/post_predictor/face_quality_post_predictor.cc
    src/post_predictor/act_post_predictor.cc
    src/post_predictor/backbone_post_predictor.cc
    src/post_predictor/vid_post_predictor.cc
    src/post_predictor/common_lmk_post_predictor.cc
    include/cnn_method/cnn_method.h
    include/cnn_method/cnn_const.h
    include/cnn_method/util/util.h
    include/cnn_method/util/model_info.h
    include/cnn_method/util/align_face.h
    include/cnn_method/util/act_data_preprocess.h
    include/cnn_method/util/cnn_method_data.h
    include/cnn_method/util/cnn_method_config.h
    include/cnn_method/predictor/predictor.h
    include/cnn_method/predictor/predictor_factory.h
    include/cnn_method/predictor/img_input_predictor.h
    include/cnn_method/predictor/lmk_input_predictor.h
    include/cnn_method/predictor/rect_input_predictor.h
    include/cnn_method/predictor/vehicle_img_input_predictor.h
    include/cnn_method/predictor/lmk_sequence_input_predictor.h
    include/cnn_method/predictor/vid_input_predictor.h
    include/cnn_method/post_predictor/post_predictor.h
    include/cnn_method/post_predictor/age_gender_post_predictor.h
    include/cnn_method/post_predictor/binary_classify_post_predictor.h
    include/cnn_method/post_predictor/antispf_post_predictor.h
    include/cnn_method/post_predictor/faceid_post_predictor.h
    include/cnn_method/post_predictor/lmk_pose_post_predictor.h
    include/cnn_method/post_predictor/post_predictor_factory.h
    include/cnn_method/post_predictor/platenum_post_predictor.h
    include/cnn_method/post_predictor/vehicle_color_post_predictor.h
    include/cnn_method/post_predictor/vehicle_type_post_predictor.h
    include/cnn_method/post_predictor/classify_post_predictor.h
    include/cnn_method/post_predictor/face_quality_post_predictor.h
    include/cnn_method/post_predictor/act_post_predictor.h
    include/cnn_method/post_predictor/backbone_post_predictor.h
    include/cnn_method/post_predictor/vid_post_predictor.h
    include/cnn_method/post_predictor/common_lmk_post_predictor.h
)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(
  ${PROJECT_SOURCE_DIR}/lib/3rdparty
  ${PROJECT_SOURCE_DIR}/example/lib
  ${HOST_PACKAGE_DIR}/bpu_predict/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/image_utils/lib
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
  ${PROJECT_SOURCE_DIR}/../../deps/hobotlog/lib
  ${PROJECT_SOURCE_DIR}/../../deps/jsoncpp/lib
  ${PROJECT_SOURCE_DIR}/../../deps/opencv/lib
  ${PROJECT_SOURCE_DIR}/../../deps/gtest/lib
)

include_directories(${PROJECT_SOURCE_DIR}/example/include)

add_library(cnn_method STATIC ${SOURCE_FILES})

add_subdirectory(test)
#add_subdirectory(example)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/cnn_method)

message("MY_OUTPUT_ROOT is ${MY_OUTPUT_ROOT}")
install(TARGETS cnn_method
        DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(FILES ${PROJECT_SOURCE_DIR}/include/cnn_method/cnn_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/cnn_method/)
