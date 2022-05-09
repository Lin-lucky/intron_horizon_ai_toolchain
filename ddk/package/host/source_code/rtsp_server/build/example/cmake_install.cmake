# Install script for directory: /home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin" TYPE EXECUTABLE FILES "/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build/example/rtsp_server_example")
  if(EXISTS "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example"
         OLD_RPATH "/root/.horizon/ddk/xj3_aarch64/xstream/lib:/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/../third_party/aarch64/hobotlog/lib:/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/../third_party/aarch64/jsoncpp/lib:/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/../third_party/aarch64/live555/lib:/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/../third_party/aarch64/opencv/lib:/root/.horizon/ddk/xj3_aarch64/appsdk/appuser/lib:/root/.horizon/ddk/xj3_aarch64/appsdk/appuser/lib/hbmedia:/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/build:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/bin/rtsp_server_example")
    endif()
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server/run.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/output/rtsp_server" TYPE FILE FILES "/home/jenkins/workspace/ess_ai_express_component_1.0.15b/rtsp_server/example/run.sh")
endif()

