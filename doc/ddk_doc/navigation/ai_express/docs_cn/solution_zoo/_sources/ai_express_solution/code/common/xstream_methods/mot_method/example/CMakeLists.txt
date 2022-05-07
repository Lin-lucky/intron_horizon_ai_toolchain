cmake_minimum_required(VERSION 2.8)

option(SHARED_LIB "build shared lib" OFF)

set(SOURCE_FILES
  ${CMAKE_CURRENT_LIST_DIR}/method_factory.cc
)

set(BASE_LIBRARIES jsoncpp xstream hobotlog rt pthread dl)

set(MOT_LIBRARIES
        #        multi_obj_tracking_lite
        #        mxnet_shared
        #        xstream_basedata_helper
        #        cudnn7
        )

#set(OpenCV_LIBS opencv_highgui opencv_imgcodecs opencv_calib3d opencv_imgproc opencv_video opencv_videoio
#        opencv_core libpng z libtiff libwebp libpng libjpeg libjasper IlmImf opencv_features2d opencv_flann
#        opencv_imgcodecs opencv_ml opencv_objdetect opencv_photo opencv_shape
#        opencv_stitching opencv_superres opencv_videostab)

if (${SHARED_LIB})
    add_library(mot_example_shared SHARED ${SOURCE_FILES})
    target_link_libraries(mot_example_shared ${CMAKE_PROJECT_NAME} -Wl,--whole-archive xstream -Wl,--no-whole-archive
            ${MOT_LIBRARIES}
#            ${OpenCV_LIBS}
            ${BASE_LIBRARIES})

    add_executable(iou_mot_example iou_mot_example.cc)
    target_link_libraries(iou_mot_example mot_example_shared)

    #add_executable(reid_mot_example reid_mot_example.cc)
    #target_link_libraries(reid_mot_example mot_example_shared)
  if (${ENABLE_IOU2}) 
    add_executable(iou_mot_example2 iou_mot_example_2.cc)
    target_link_libraries(iou_mot_example2 mot_example_shared)
  endif()
else()
    add_executable(iou_mot_example ${SOURCE_FILES} iou_mot_example.cc)
    target_link_libraries(iou_mot_example
            mot_method
#            ${OpenCV_LIBS}
            ${BASE_LIBRARIES})

  if (${ENABLE_IOU2})
    add_executable(iou_mot_example2 ${SOURCE_FILES} iou_mot_example_2.cc)
    target_link_libraries(iou_mot_example2
           mot_method
           ${BASE_LIBRARIES})
  endif()
endif()
