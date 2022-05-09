include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

message(${CMAKE_CURRENT_SOURCE_DIR})

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${HOST_PACKAGE_DIR}/appsdk/appuser/lib/hbmedia

)

set(SOURCE_FILES
	src/utils.h
	src/sample.cc
)

add_executable(roi_resizer_task ${SOURCE_FILES})

set(X3_HAPI_LIB vio cam uvc isp iar isp_algo tinyalsa multimedia z avformat hbmedia)

target_link_libraries(roi_resizer_task ${PROJECT_NAME} video_source ${X3_HAPI_LIB})
