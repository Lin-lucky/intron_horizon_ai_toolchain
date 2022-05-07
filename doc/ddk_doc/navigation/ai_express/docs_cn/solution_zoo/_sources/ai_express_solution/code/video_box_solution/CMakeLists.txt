cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
message("tool chain:${CMAKE_CXX_COMPILER}")

option(PARENT_BUILD "build subdirectory from here" ON)

project(video_box)

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

message("solution_zoo common dir:${SOLUTION_ZOO_DIR}")
# get deps directory end

set(CMAKE_CXX_STANDARD 11)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

# 头文件路径信息
set(PROJECT_INC
  "include/plot_smart_data.h"
  "include/venc_module.h"
  "include/video_box_common.h"
  "include/video_processor.h"
  "include/vot_module.h"
  "include/message/feature_frame_message/feature_frame_message.h"
  "include/message/feature_frame_message/feature_info.h"
  "include/message/feature_frame_message/float_array.h"
  "include/message/smart_feature_message/smart_feature_message.h"
  "include/xware_plugin/json_config_wrapper.h"
  "include/xware_plugin/ware_db.h"
  "include/xware_plugin/ware_message.h"
  "include/xware_plugin/ware_plugin.h"
)

# 源文件路径信息
set(PROJECT_SRC
  "src/main.cc"
  "src/method_factory.cc"
  "src/media_pipe_manager/media_pipe_manager.cc"
  "src/media_pipe_manager/media_pipeline.cc"
  "src/media_pipe_manager/vdec_module.cc"
  "src/media_pipe_manager/vps_module.cc" 
  "src/rtsp_client/audio_g711_sink.cc"
  "src/rtsp_client/h264_sink.cc"
  "src/rtsp_client/h265_sink.cc"
  "src/rtsp_client/hevc_sps.cc"
  "src/rtsp_client/rtsp_client.cc"
  "src/rtsp_client/sps_pps.cc"
  "src/rtsp_client/sps_info_mgr.cc"
  "src/rtsp_plugin/rtsp_message.cc"
  "src/rtsp_plugin/rtsp_plugin.cc"
  "src/smart_plugin/convert.cc"
  "src/smart_plugin/display_info.cc"
  "src/smart_plugin/plot_smart_data.cc"
  "src/smart_plugin/runtime_monitor.cc"
  "src/smart_plugin/smart_plugin.cc"
  "src/smart_plugin/util.cc"
  "src/smart_plugin/utils_box.cc"
  "src/smart_plugin/vision_utils.cc"
  "src/smart_plugin/venc_module.cc"
  "src/smart_plugin/video_processor.cc"
  "src/smart_plugin/vot_module.cc"
  "src/utils/time_helper.cc"
  # ware_plugin
  "src/xware_plugin/ware_db.cc"
  "src/xware_plugin/ware_plugin.cc"
  "src/xware_plugin/ware_message.cc"
  ${HOST_PACKAGE_DIR}/xproto/include/xproto/msg_type/protobuf/x3.pb.h
  ${HOST_PACKAGE_DIR}/xproto/include/xproto/msg_type/protobuf/x3.pb.cc
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/message/include/protobuf/x3ware.pb.h
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/message/include/protobuf/x3ware.pb.cc
)

set(SOURCE_FILES
  ${PROJECT_SRC}
  ${PROJECT_INC}
)

include_directories(
  # system image
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/
  ${HOST_PACKAGE_DIR}/appsdk/appuser/include/vio/
  # bpu_predict
  ${HOST_PACKAGE_DIR}/bpu_predict/include/
  # xstream
  ${HOST_PACKAGE_DIR}/xstream/include/
    # model_inference
    ${HOST_PACKAGE_DIR}/model_inference/include/
    # xstream methods
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/mot_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/filter_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/merge_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/vote_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/facesnap_filter_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/snapshot_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/grading_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_predict_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_postprocess_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/dnn_postprocess_method/include/
    ${SOLUTION_ZOO_DIR}/common/xstream_methods/dnn_predict_method/include/

  # xproto
  ${HOST_PACKAGE_DIR}/xproto/include/
  # xproto plugins messsage
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/message/include/
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/visual_plugin/include/
  # blocking queue
  ${SOLUTION_ZOO_DIR}/common/

  # third_party
  ${SOLUTION_ZOO_DIR}/common/deps/hobotlog/include/
  ${SOLUTION_ZOO_DIR}/common//deps/jsoncpp/include/
  ${SOLUTION_ZOO_DIR}/common/deps/opencv/include/
  ${SOLUTION_ZOO_DIR}/common/deps/live555/include/
  ${SOLUTION_ZOO_DIR}/common/deps/aes/include/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/include/
  ${SOLUTION_ZOO_DIR}/common/deps/xwarehouse/include/
  ${SOLUTION_ZOO_DIR}/common/deps/libyuv/include/
  ${SOLUTION_ZOO_DIR}/common/deps/libjpeg-turbo/include/
  ${SOLUTION_ZOO_DIR}/common/blocking_queue/

  include
  include/message/
)

link_directories(
  # system image
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu/
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbbpu/
  # bpu_predict
  ${HOST_PACKAGE_DIR}/bpu_predict/lib/
  # xstream
  ${HOST_PACKAGE_DIR}/xstream/lib/
  # xproto
  ${HOST_PACKAGE_DIR}/xproto/lib/
  # image_utils
  ${HOST_PACKAGE_DIR}/image_utils/lib/
  #model_inference
  ${HOST_PACKAGE_DIR}/model_inference/lib/
  # third_party
  ${SOLUTION_ZOO_DIR}/common/deps/hobotlog/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/jsoncpp/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/opencv/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/live555/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/xwarehouse/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/aes/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/libjpeg-turbo/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/ipc_tracking/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/iou_based_mot/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/libyuv/lib/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/lib
  ${SOLUTION_ZOO_DIR}/common/deps/zeroMQ/lib/
  ${HOST_PACKAGE_DIR}/dnn/lib/

  ${SOLUTION_ZOO_DIR}/common/wrapper/media_codec/output/xstream_media_codec/lib/
)

# treat warnings as errors
# set(CMAKE_CXX_FLAGS  "-Werror ${CMAKE_CXX_FLAGS}"  )

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)
message("MY_OUTPUT_ROOT is ${MY_OUTPUT_ROOT}")

add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/mot_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/mot_method/output/
)

add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/merge_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/merge_method/output/
)
add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/vote_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/vote_method/output/
)
add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/facesnap_filter_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/facesnap_filter_method/output/
)
add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/snapshot_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/snapshot_method/output/
)
add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/grading_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/grading_method/output/
)
add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_predict_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_predict_method/output/
)
add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_postprocess_method/
  ${SOLUTION_ZOO_DIR}/common/xstream_methods/multitask_postprocess_method/output/
)

add_subdirectory(
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/visual_plugin/
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/visual_plugin/output/
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

set(BASE_LIBRARIES
  visual_plugin
  xstream
  xproto
  mot_method 
  merge_method vote_method
  facesnap_filter_method snapshot_method grading_method
  multitask_predict_method
  multitask_postprocess_method
  protobuf
  model_inference
  bpu_predict ipc_tracking image_utils hobotlog jsoncpp opencv_world vio cam pthread dnn
)

set(LIVE_LIB BasicUsageEnvironment groupsock liveMedia UsageEnvironment turbojpeg)

set(X3_HAPI_LIB isp iar isp_algo tinyalsa multimedia hbmedia z avformat )

target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${LIVE_LIB} ${X3_HAPI_LIB} xwarehouse )

install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/configs/
        DESTINATION ${MY_OUTPUT_ROOT}/configs/)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${MY_OUTPUT_ROOT}/include)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/)
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/data/
        DESTINATION ${MY_OUTPUT_ROOT}/data/)
 