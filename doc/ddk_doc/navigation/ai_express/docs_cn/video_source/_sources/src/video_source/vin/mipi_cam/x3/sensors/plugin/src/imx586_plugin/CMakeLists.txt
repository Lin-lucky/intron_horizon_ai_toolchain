cmake_minimum_required(VERSION 2.8)

# 项目名称
project(imx586_plugin)

# 源文件路径信息
file(GLOB PROJECT_SRC
  "*.cc"
  )

set(SOURCE_FILES ${PROJECT_SRC})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

set(BASE_LIBRARIES dl)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT}/lib)

