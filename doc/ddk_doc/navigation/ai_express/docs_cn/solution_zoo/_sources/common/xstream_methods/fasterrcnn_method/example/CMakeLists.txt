cmake_minimum_required(VERSION 2.8)

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/include
)

set(BASE_LIBRARIES jsoncpp pthread dl rt)

set(OpenCVLib opencv_world)
set(VioWrapperLib viowrapper)
set(BASE_LIBRARIES hobotlog ${BASE_LIBRARIES} ${OpenCVLib})

set(XSTREAM_LB -Wl,--whole-archive xstream -Wl,--no-whole-archive)

add_executable(FasterRCNNMethod_example
        main.cc
        method_factory.cc
        dump_util.h
        stop_watch.cc
        test_model_info.cc
        test_faster_rcnn_image.cc
        #test_fb_pyramid.cc
        #test_fb_fasterrcnn.cc
        dump_nv12.cc
        test_hbcc_info.cc
        #test_two_fasterrcnn.cc
        dump_face_result.cc
        )

target_link_libraries(FasterRCNNMethod_example
        fasterrcnn_method
        #${VioWrapperLib}
        ${XSTREAM_LB}
        ${DYNAMIC_DEPS_LIBS}
        hobotlog
        jsoncpp
        ${OpenCVLib}
        ${OpenCVDeps}
        -lpthread
        -ldl
        -lrt
        )

