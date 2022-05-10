cmake_minimum_required(VERSION 2.8)
set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
project(yolov3_mobilenetv2_example)

option(PARENT_BUILD "build subdirectory from here" ON)
option(RELEASE_LIB "build version of release" ON)
# get deps directory begin
set(SOLUTION_ZOO_DIR)
	function(get_parent_dir arg dir n)
		foreach(i RANGE ${ARGV2})
			get_filename_component(dir ${dir} DIRECTORY)
			# message("i:${i} dir:${dir}")
		endforeach(i)
		set(SOLUTION_ZOO_DIR ${dir} PARENT_SCOPE)
	endfunction()
get_parent_dir(XSTREAM_CURRENT_DIR ${PROJECT_SOURCE_DIR} 1)
message("solution_zoo dir:${SOLUTION_ZOO_DIR}")

set(WEB_DISPLAY_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/web_display_plugin)
set(SMART_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/smart_plugin)
set(VISUAL_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/visual_plugin)


set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
add_subdirectory(${WEB_DISPLAY_PLUGIN_SOURCE}
  ${WEB_DISPLAY_PLUGIN_SOURCE}/output
)

add_subdirectory(${SMART_PLUGIN_SOURCE}
  ${SMART_PLUGIN_SOURCE}/output
)

add_subdirectory(${VISUAL_PLUGIN_SOURCE}
  ${VISUAL_PLUGIN_SOURCE}/output
)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
    # system image
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/cam
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio
    # video_source_plugin
    ${HOST_PACKAGE_DIR}/video_source/include/
    # model_inference
    ${HOST_PACKAGE_DIR}/model_inference/include/
    # bpu_predict
    ${HOST_PACKAGE_DIR}/bpu_predict/include/
    # xstream
    ${HOST_PACKAGE_DIR}/xstream/include/
    # xproto
    ${HOST_PACKAGE_DIR}/xproto/include/
    # message
    ${SOLUTION_ZOO_DIR}/common/xproto_plugins/message/include/

  # smart_plugin
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/smart_plugin/include
  # web_display_plugin
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/web_display_plugin/include

  # solution common
  ${SOLUTION_ZOO_DIR}/common/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/include/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/dnn_postprocess_method/include
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/dnn_predict_method/include
  ${SOLUTION_ZOO_DIR}/common/blocking_queue

  # third_party
  ${SOLUTION_ZOO_DIR}/common/deps/hobotlog/include/
  ${SOLUTION_ZOO_DIR}/common/deps/jsoncpp/include/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/include/
  ${SOLUTION_ZOO_DIR}/common/deps/opencv/include/
)

link_directories(
  ${SOLUTION_ZOO_DIR}/common/deps/hobotlog/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/jsoncpp/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/opencv/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/libjpeg-turbo/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/gtest/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/uWS/lib
  ${HOST_PACKAGE_DIR}/xproto/lib
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${HOST_PACKAGE_DIR}/image_utils/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
  ${HOST_PACKAGE_DIR}/bpu_predict/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
  ${HOST_PACKAGE_DIR}/video_source/lib/
  ${HOST_PACKAGE_DIR}/model_inference/lib/
  ${HOST_PACKAGE_DIR}/dnn/lib/

)

set(SOURCE_FILES
    main.cc
    method_factory.cc
    plugins/example_smart_plugin/src/example_smart_plugin.cc
    plugins/example_web_display_plugin/src/example_web_display_plugin.cc
)

include_directories(
    plugins/example_smart_plugin/include/
    plugins/example_web_display_plugin/include/
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

set(BASE_LIBRARIES smart_plugin web_display_plugin
        bpu_predict xstream xproto model_inference
        image_utils hobotlog jsoncpp opencv_world video_source_plugin dl rt dnn)

target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/configs/
        DESTINATION ${MY_OUTPUT_ROOT}/configs/)
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/models/
        DESTINATION ${MY_OUTPUT_ROOT}/models/)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/)
