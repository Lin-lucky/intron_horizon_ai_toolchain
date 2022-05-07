cmake_minimum_required(VERSION 2.8)
set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)

project(multitask_perception)

option(PARENT_BUILD "build subdirectory from here" ON)
option(RELEASE_LIB "build version of release" ON)
option(TEST_NEW_SMART_MESSAGE "test SmartLegibleMessage" OFF)
if (${TEST_NEW_SMART_MESSAGE})
    add_definitions(-DTEST_NEW_MESSAGE)
endif()
# get deps directory begin
set(SOLUTION_ZOO_DIR)
	function(get_parent_dir arg dir n)
		foreach(i RANGE ${ARGV2})
			get_filename_component(dir ${dir} DIRECTORY)
			# message("i:${i} dir:${dir}")
		endforeach(i)
		set(SOLUTION_ZOO_DIR ${dir} PARENT_SCOPE)
	endfunction()
get_parent_dir(XSTREAM_CURRENT_DIR ${PROJECT_SOURCE_DIR} 0)

message("solution_zoo common dir:${SOLUTION_ZOO_COMMON_DIR}")

set(WEB_DISPLAY_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/web_display_plugin)
set(SMART_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/smart_plugin)
set(VISUAL_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/visual_plugin)
set(UVC_DISPLAY_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/uvc_display_plugin)
set(UVC_SERVER_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/uvc_server_plugin)
set(RTSP_SERVER_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/rtsp_server_plugin)
if(${TEST_NEW_SMART_MESSAGE})
set(EXAMPLE_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/example_plugin)
endif()
set(MOT_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/mot_method)
set(MERGE_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/merge_method)
set(VOTE_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/vote_method)
set(GRADING_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/grading_method)
set(FILTER_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/filter_method)
set(SNAPSHOT_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/snapshot_method)
set(MULTITASK_POSTPROCESS_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_postprocess_method)
set(MULTITASK_PREDICT_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_predict_method)
set(VEHICLE_COLOR_POSTPROCESS_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_color_postprocess_method)
set(VEHICLE_COLOR_PREDICT_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_color_predict_method)
set(RECT_INPUT_PREDICT_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/rect_input_predict_method)
set(PLATE_NUM_POSTPROCESS_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/plate_num_postprocess_method)
set(VEHICLE_PLATE_MATCH_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_plate_match_method)
set(PLATE_VOTE_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/plate_vote_method)
set(VEHICLE_TYPE_POSTPROCESS_METHOD_SOURCE ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_type_postprocess_method)

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

add_subdirectory(${UVC_DISPLAY_PLUGIN_SOURCE}
  ${UVC_DISPLAY_PLUGIN_SOURCE}/output
)

add_subdirectory(${UVC_SERVER_PLUGIN_SOURCE}
  ${UVC_SERVER_PLUGIN_SOURCE}/output
)

add_subdirectory(${RTSP_SERVER_PLUGIN_SOURCE}
  ${RTSP_SERVER_PLUGIN_SOURCE}/output
)

if(${TEST_NEW_SMART_MESSAGE})
add_subdirectory(${EXAMPLE_PLUGIN_SOURCE}
  ${EXAMPLE_PLUGIN_SOURCE}/output
)
endif()

add_subdirectory(${MERGE_METHOD_SOURCE}
  ${MERGE_METHOD_SOURCE}/output
)

add_subdirectory(${MOT_METHOD_SOURCE}
  ${MOT_METHOD_SOURCE}/output
)

add_subdirectory(${VOTE_METHOD_SOURCE}
  ${VOTE_METHOD_SOURCE}/output
)

add_subdirectory(${GRADING_METHOD_SOURCE}
  ${GRADING_METHOD_SOURCE}/output
)

add_subdirectory(${FILTER_METHOD_SOURCE}
  ${FILTER_METHOD_SOURCE}/output
)

add_subdirectory(${SNAPSHOT_METHOD_SOURCE}
  ${SNAPSHOT_METHOD_SOURCE}/output
)

add_subdirectory(${MULTITASK_POSTPROCESS_METHOD_SOURCE}
  ${MULTITASK_POSTPROCESS_METHOD_SOURCE}/output
)

add_subdirectory(${MULTITASK_PREDICT_METHOD_SOURCE}
  ${MULTITASK_PREDICT_METHOD_SOURCE}/output
)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()
message("HOST_PACKAGE_DIR: ${HOST_PACKAGE_DIR}")

add_subdirectory(${RECT_INPUT_PREDICT_METHOD_SOURCE}
  ${RECT_INPUT_PREDICT_METHOD_SOURCE}/output
)

add_subdirectory(${VEHICLE_COLOR_POSTPROCESS_METHOD_SOURCE}
  ${VEHICLE_COLOR_POSTPROCESS_METHOD_SOURCE}/output
)

add_subdirectory(${VEHICLE_COLOR_PREDICT_METHOD_SOURCE}
  ${VEHICLE_COLOR_PREDICT_METHOD_SOURCE}/output
)

add_subdirectory(${PLATE_NUM_POSTPROCESS_METHOD_SOURCE}
  ${PLATE_NUM_POSTPROCESS_METHOD_SOURCE}/output
)

add_subdirectory(${VEHICLE_PLATE_MATCH_METHOD_SOURCE}
  ${VEHICLE_PLATE_MATCH_METHOD_SOURCE}/output
)

add_subdirectory(${PLATE_VOTE_METHOD_SOURCE}
  ${PLATE_VOTE_METHOD_SOURCE}/output
)

add_subdirectory(${VEHICLE_TYPE_POSTPROCESS_METHOD_SOURCE}
  ${VEHICLE_TYPE_POSTPROCESS_METHOD_SOURCE}/output
)

include_directories(
    # system image
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/cam
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio
    ${HOST_PACKAGE_DIR}/appsdk/appuser/include/alsa
    ${HOST_PACKAGE_DIR}/rtsp_server/include/
    ${HOST_PACKAGE_DIR}/video_source/include/
    ${HOST_PACKAGE_DIR}/model_inference/include/
    ${HOST_PACKAGE_DIR}/uvc_server/include/
    # bpu_predict
    ${HOST_PACKAGE_DIR}/bpu_predict/include/
    # xstream
    ${HOST_PACKAGE_DIR}/xstream/include/
    # xproto
    ${HOST_PACKAGE_DIR}/xproto/include/
    # message
    ${SOLUTION_ZOO_DIR}/common/xproto_plugins/message/include/
    # xstream methods
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/mot_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/filter_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/merge_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/lowpass_filter_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/vote_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/facesnap_filter_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/snapshot_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/grading_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/dnn_postprocess_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/dnn_predict_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_postprocess_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_predict_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/rect_input_predict_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_color_postprocess_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_color_predict_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/plate_num_postprocess_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_plate_match_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/plate_vote_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/vehicle_type_postprocess_method/include/

  # xproto
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/smart_plugin/include
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/web_display_plugin/include
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/visual_plugin/include
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/uvc_display_plugin/include
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/uvc_server_plugin/include
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/rtsp_server_plugin/include
  ${SOLUTION_ZOO_DIR}/common/blocking_queue
  ${SOLUTION_ZOO_DIR}/common/thread_pool
  ${SOLUTION_ZOO_DIR}/common/wrapper/audio_source/alsa_device/include
  ${SOLUTION_ZOO_DIR}/common/wrapper/media_codec/include
  ${SOLUTION_ZOO_DIR}/common/

  # third_party
  ${SOLUTION_ZOO_DIR}/common/deps/hobotlog/include/
  ${SOLUTION_ZOO_DIR}/common/deps/jsoncpp/include/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/include/
  ${SOLUTION_ZOO_DIR}/common/deps/opencv/include/
  ${SOLUTION_ZOO_DIR}/common/deps/libjpeg-turbo/include/
  ${SOLUTION_ZOO_DIR}/common/deps/zeroMQ/include/
)
if(${TEST_NEW_SMART_MESSAGE})
include_directories(${SOLUTION_ZOO_DIR}/common/xproto_plugins/example_plugin/include)
endif()
link_directories(
  ${SOLUTION_ZOO_DIR}/common/deps/hobotlog/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/jsoncpp/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/opencv/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/uWS/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/ipc_tracking/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/gtest/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/live555/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/libjpeg-turbo/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/iou_based_mot/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/zeroMQ/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/libjpeg-turbo/lib/
  ${HOST_PACKAGE_DIR}/xproto/lib
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${HOST_PACKAGE_DIR}/image_utils/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia
  ${HOST_PACKAGE_DIR}/bpu_predict/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu
  ${HOST_PACKAGE_DIR}/rtsp_server/lib/
  ${HOST_PACKAGE_DIR}/video_source/lib/
  ${HOST_PACKAGE_DIR}/uvc_server/lib/
  ${HOST_PACKAGE_DIR}/model_inference/lib/
  ${HOST_PACKAGE_DIR}/dnn/lib/
)
set(SOURCE_FILES
        src/main.cc
        src/method_factory.cc
        )

add_executable(multitask_perception ${SOURCE_FILES})

set(BASE_LIBRARIES smart_plugin visual_plugin rtsp_server_plugin model_inference
        bpu_predict filter_method merge_method snapshot_method grading_method vote_method
        mot_method multitask_predict_method multitask_postprocess_method
        rect_input_predict_method vehicle_color_postprocess_method vehicle_color_predict_method
        plate_num_postprocess_method vehicle_plate_match_method plate_vote_method
        vehicle_type_postprocess_method ipc_tracking xstream hobotlog
        jsoncpp opencv_world web_display_plugin video_source_plugin uvc_server_plugin xproto dnn)
if(NOT ${TEST_NEW_SMART_MESSAGE})
  target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} example_plugin)
endif()
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/configs/
        DESTINATION ${MY_OUTPUT_ROOT}/configs/)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/)
