cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(mot_method)

message("${PROJECT_SOURCE_DIR}")
option(ENABLE_IOU2 "build iou 2.0" ON)
if(${ENABLE_IOU2})
    add_definitions(-DENABLE_IOU2)
endif()

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
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/ipc_tracking/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/iou_based_mot/include
  ${HOST_PACKAGE_DIR}/xstream/include
)

link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/opencv/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/ipc_tracking/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/iou_based_mot/lib
)

set(BASE_LIBRARIES hobotlog rt dl pthread)

if (${ENABLE_IOU2})
    set(IOU2_SRC_FILE "src/method/mot/iou2.cc")
    set(IOU2_DEP_LIB "iou_based_mot")
endif()

set(SOURCE_FILES
        src/method/mot_method.cc
        src/method/mot/iou.cc
        ${IOU2_SRC_FILE}
)

set(MOT_LIBRARIES
         ipc_tracking
         ${IOU2_DEP_LIB}
        )
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
                      ${MOT_LIBRARIES}
#                      ${OpenCV_LIBS}
                      ${BASE_LIBRARIES} )

if(NOT ${PARENT_BUILD})
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
endif()

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES include/${PROJECT_NAME}/mot_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

add_subdirectory(test)
add_subdirectory(example)
