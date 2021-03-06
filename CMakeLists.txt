cmake_minimum_required(VERSION 2.8)

project(your_project_name)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG " -Wall  -g -O0 ")
set(CMAKE_C_FLAGS_DEBUG " -Wall  -g -O0 ")
set(CMAKE_CXX_FLAGS_RELEASE " -Wall -O3 ")
set(CMAKE_C_FLAGS_RELEASE " -Wall -O3 ")#-Werror


if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# define dnn lib path
set(DNN_PATH "~/.horizon/ddk/xj3_aarch64/dnn/")
set(APPSDK_PATH "~/.horizon/ddk/xj3_aarch64/appsdk/appuser/")
set(OPENCV_PATH "/open_explorer/ddk/samples/ai_benchmark/code/deps/aarch64/opencv")
#set(GLOG_PATH "/open_explorer/ddk/samples/ai_benchmark/code/deps/aarch64/glog")
#set(GFLAG_PATH "/open_explorer/ddk/samples/ai_benchmark/code/deps/aarch64/gflags")


set(DNN_LIB_PATH ${DNN_PATH}/lib)
set(APPSDK_LIB_PATH ${APPSDK_PATH}/lib/hbbpu)
set(BPU_libs dnn cnn_intf hbrt_bernoulli_aarch64)

include_directories(${DNN_PATH}/include
                    ${APPSDK_PATH}/include
                    ${OPENCV_PATH}/include
#                    ${GLOG_PATH}/include
#                    ${GFLAG_PATH}/include
                    )
link_directories(${DNN_LIB_PATH}
                ${APPSDK_PATH}/lib/hbbpu
                ${APPSDK_PATH}/lib
                ${OPENCV_PATH}/lib
#                {GLOG_PATH}/lib
#                {GFLAG_PATH}/lib
                )
#test_main
#horizon
add_executable(horizon horizon.cpp)
target_link_libraries(horizon
                      ${BPU_libs}
                      opencv_world
                      pthread
#                      gflags
#                      glog
                      rt
                      dl)
