cmake_minimum_required(VERSION 2.8)

option(PARENT_BUILD "is build from parent" OFF)
option(COVERAGE "Code Coverage " OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

PROJECT(model_inference)

option(RELEASE_LIB "build release version" ON)

if (COVERAGE)
    message("compile model_inference for coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O0 -g -fprofile-arcs -ftest-coverage")
endif ()

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
    include
    ${HOST_PACKAGE_DIR}/bpu_predict/include
    ${HOST_PACKAGE_DIR}/xstream/include
    ${HOST_PACKAGE_DIR}/image_utils/include
    ${HOST_PACKAGE_DIR}/dnn/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/libyuv/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/opencv/include
)

link_directories(
    ${HOST_PACKAGE_DIR}/bpu_predict/lib/
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
    ${HOST_PACKAGE_DIR}/xstream/lib
    ${HOST_PACKAGE_DIR}/image_utils/lib
    ${HOST_PACKAGE_DIR}/dnn/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/hobotlog/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/jsoncpp/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/libyuv/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/opencv/lib
)

set(SOURCE_FILES
	src/inferencer.cc
	src/inference_engine.cc
	src/inference_engine_bpu.cc
  src/inference_engine_dnn.cc
  src/preprocess.cc
  src/preprocess/faceid_preprocess.cc
  src/preprocess/image_preprocess.cc
  src/preprocess/pyramid_preprocess.cc
  src/preprocess/pyramid_roi_bpu_preprocess.cc
  src/preprocess/pyramid_roi_resizer_preprocess.cc
  src/preprocess/pyramid_roi_preprocess.cc
  src/preprocess/gesture_preprocess.cc
  src/preprocess/utils/image_process.cc
  src/preprocess/utils/roi_process.cc
  src/preprocess/utils/lmks_process.cc
  src/postprocess.cc
  src/postprocess/age_gender_postprocess.cc
  src/postprocess/face_quality_postprocess.cc
  src/postprocess/faceid_postprocess.cc
  src/postprocess/horizon_multitask_postprocess.cc
  src/postprocess/yolov3_postprocess.cc
  src/postprocess/lmks3_postprocess.cc
  src/postprocess/lmks4_postprocess.cc
  src/postprocess/vehicle_color_postprocess.cc
  src/postprocess/vehicle_type_postprocess.cc
  src/postprocess/plate_num_postprocess.cc
  src/postprocess/mobilenetv2_postprocess.cc
  src/postprocess/gesture_postprocess.cc
  src/postprocess/matrix_detection_postprocess.cc
  src/postprocess/utils/gesture_postprocess_util.cc
)

if(${RELEASE_LIB})
    message("build release version")
    set(CMAKE_BUILD_TYPE Release)
else()
    message("build debug version")
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC pthread rt dl jsoncpp hobotlog
    libyuv.a opencv_world image_utils
    xstream bpu_predict cnn_intf dnn hbrt_bernoulli_aarch64)

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include/)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)


add_subdirectory(test/tensor_task)
add_subdirectory(test/roi_task)
