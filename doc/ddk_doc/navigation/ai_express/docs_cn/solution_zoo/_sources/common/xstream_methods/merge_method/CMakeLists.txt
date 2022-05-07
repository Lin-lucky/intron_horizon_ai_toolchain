cmake_minimum_required(VERSION 2.8)
option(PARENT_BUILD "is build from parent" OFF)
if(NOT ${PARENT_BUILD})
  set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
endif()
message("tool chain:${CMAKE_CXX_COMPILER}")

project(merge_method)

message("${PROJECT_SOURCE_DIR}")
option(SUPPORT_RGB_NIR_MERGE "support rgb nir merge" OFF)
if (${SUPPORT_RGB_NIR_MERGE})
    add_definitions(-DRGB_NIR_MERGE)
endif ()

STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}) 

#message(${CMAKE_CURRENT_SOURCE_DIR})
if(DEFINED ENV{HORIZON_LIB_PATH})
  set(HOST_PACKAGE_DIR $ENV{HORIZON_LIB_PATH}/ddk/xj3_aarch64)
else()
  set(HOST_PACKAGE_DIR ~/.horizon/ddk/xj3_aarch64)
endif()

include_directories(
  include
  include/merge_method
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/include
  ${HOST_PACKAGE_DIR}/xstream/include
)

set(SOURCE_FILES
        src/method/merge_method.cc
        src/method/data_type/data_type.cc
        src/method/strategy/head_face.cc
        src/method/strategy/head_body.cc
        src/method/strategy/rgb_nir.cc
        )

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

link_directories(
  ${HOST_PACKAGE_DIR}/xstream/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/hobotlog/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/jsoncpp/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/opencv/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/deps/gtest/lib
)

set(BASE_LIBRARIES xstream hobotlog jsoncpp opencv_world)
if (${SUPPORT_RGB_NIR_MERGE})
    set(BASE_LIBRARIES ${BASE_LIBRARIES} dual_camera_algo vio cam)
endif ()

target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/${PROJECT_NAME}/)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/merge_method/merge_method.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/merge_method)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/merge_method/data_type/data_type.h
        DESTINATION ${MY_OUTPUT_ROOT}/include/merge_method/data_type)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${MY_OUTPUT_ROOT}/lib)

add_subdirectory(test)
add_subdirectory(example)
