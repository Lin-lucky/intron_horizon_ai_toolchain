cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)

option(PARENT_BUILD "build subdirectory from here" ON)

# 项目名称
project(example_web_display_plugin)

# get deps directory begin
set(SOLUTION_ZOO_DIR)
	function(get_parent_dir arg dir n)
		foreach(i RANGE ${ARGV2})
			get_filename_component(dir ${dir} DIRECTORY)
			# message("i:${i} dir:${dir}")
		endforeach(i)
		set(SOLUTION_ZOO_DIR ${dir} PARENT_SCOPE)
	endfunction()
get_parent_dir(XSTREAM_CURRENT_DIR ${PROJECT_SOURCE_DIR} 3)

message("solution_zoo common dir:${SOLUTION_ZOO_DIR}")

set(WEB_DISPLAY_PLUGIN_SOURCE ${SOLUTION_ZOO_DIR}/common/xproto_plugins/web_display_plugin)

add_subdirectory(${WEB_DISPLAY_PLUGIN_SOURCE}
  ${WEB_DISPLAY_PLUGIN_SOURCE}/output
)

if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
  # xstream
  ${HOST_PACKAGE_DIR}/xstream/include/
  # xproto
  ${HOST_PACKAGE_DIR}/xproto/include/
  # message
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/message/include/
  # solution common
  ${SOLUTION_ZOO_DIR}/common/
  ${SOLUTION_ZOO_DIR}/common/deps/protobuf/include/
  # web_display_plugin
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/web_display_plugin/include
  # smart_plugin
  ${SOLUTION_ZOO_DIR}/common/xproto_plugins/smart_plugin/include

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
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib
  ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia/
)


# 添加头文件路径
include_directories(
    include
)

file(GLOB PROJECT_SRC
    "src/*.cc"
)

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRC})

set(BASE_LIBRARIES xstream xproto web_display_plugin
      hobotlog jsoncpp protobuf opencv_world)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/configs/
        DESTINATION ${MY_OUTPUT_ROOT}/configs)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/
        DESTINATION ${MY_OUTPUT_ROOT}/models)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/)



