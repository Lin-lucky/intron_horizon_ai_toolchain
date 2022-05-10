include_directories(
        include
)


set(SOURCE_FILES
        gtest_main.cc
        cnn_test.cc
        )

set(COMMON_DEPS
   -lpthread -ldl
    )

set(BASE_LIBRARIES jsoncpp pthread dl rt)
set(OpenCVLib opencv_world)
set(XSTREAM_LB -Wl,--whole-archive xstream -Wl,--no-whole-archive)
#set(X2_PREBUILT_LIB bpu_predict libcam.so libvio.so libcnn_intf.so)
set(X2_PREBUILT_LIB bpu_predict libcnn_intf.so)
set(VioWrapperLib )
add_executable(CNNMethod_unit_test ${SOURCE_FILES})

if (${PLATFORM_WIN})
     set(CMAKE_CXX_FLAGS_RELEASE "/MT")
     set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
     target_link_libraries(CNNMethod_unit_test debug gtestd)
     target_link_libraries(CNNMethod_unit_test optimized gtest)
else()
     target_link_libraries(CNNMethod_unit_test
                           ${COMMON_DEPS}
                           gtest
                           cnn_method
                           ${VioWrapperLib}
                           ${XSTREAM_LB}
                           ${X2_PREBUILT_LIB}
                           ${BASE_LIBRARIES}
                           ${OpenCVLib}
                           hobotlog
                           hbrt_bernoulli_aarch64
                           image_utils)
endif()

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/test)
install(TARGETS CNNMethod_unit_test
        DESTINATION ${MY_OUTPUT_ROOT})