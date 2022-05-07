get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

include_directories(${PARENT_DIR}/example)

set(TEST_SOURCE_SRC
        src/gtest_main.cc
        src/gtest_basic.cc
        src/gtest_detect.cc
        src/gtest_imgutil.cc
        ../example/method_factory.cc
        )

set(BASE_LIBRARIES jsoncpp pthread dl rt)

#set(OpenCVLib opencv_world viowrapper)
set(OpenCVLib opencv_world)
# set(OpenCVDeps libprotobuf libjpeg-turbo libwebp libpng libtiff libjasper zlib tegra_hal quirc -ldl -lm -lpthread -lrt)

set(BASE_LIBRARIES hobotlog ${BASE_LIBRARIES} ${OpenCVLib} ${OpenCVDeps})

set(XSTREAM_LB -Wl,--whole-archive xstream -Wl,--no-whole-archive)

add_executable(gtest_fasterrcnn ${TEST_SOURCE_SRC})

target_link_libraries(gtest_fasterrcnn
        fasterrcnn_method
        ${XSTREAM_LB}
        ${DYNAMIC_DEPS_LIBS}
        hobotlog
        jsoncpp
        ${OpenCVLib}
        ${OpenCVDeps}
        -lpthread
        -ldl
        -lrt
        gtest)
install(TARGETS gtest_fasterrcnn
        DESTINATION ${MY_OUTPUT_ROOT}/test)