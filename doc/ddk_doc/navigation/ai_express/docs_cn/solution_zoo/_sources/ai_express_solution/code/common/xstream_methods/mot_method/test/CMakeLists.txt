cmake_minimum_required(VERSION 2.8)

option(SHARED_LIB "build shared lib" OFF)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
)

set(SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/method_factory.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/callback.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/iou_test.cc
)

set(BASE_LIBRARIES jsoncpp.a xstream hobotlog rt pthread dl)


if (${SHARED_LIB})
    set(MOT_LIBRARIES
            #        multi_obj_tracking_lite
            #        mxnet_shared
            #        xstream_basedata_helper
            #        cudnn7
            )
    add_library(mot_gtest_shared SHARED ${SOURCE_FILES})
    target_link_libraries(mot_gtest_shared ${PROJECT_NAME} -Wl,--whole-archive xstream -Wl,--no-whole-archive
            #${MOT_LIBRARIES}
            #        ${OpenCV_LIBS}
            ${BASE_LIBRARIES})

    add_executable(gtest_mot gtest_mot_method.cc)
    target_link_libraries(gtest_mot mot_gtest_shared pthread gtest)
else()
    add_executable(gtest_mot ${SOURCE_FILES} gtest_mot_method.cc)
    target_link_libraries(gtest_mot
            mot_method
            #            ${OpenCV_LIBS}
            ${BASE_LIBRARIES}
            pthread
            gtest)
endif()
