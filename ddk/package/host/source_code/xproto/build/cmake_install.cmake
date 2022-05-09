# Install script for directory: /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/plugin/xplugin.h;/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/plugin/xpluginasync.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/plugin" TYPE FILE FILES
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/plugin/xplugin.h"
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/plugin/xpluginasync.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/message/flowmsg.h;/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/message/msg_registry.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/message" TYPE FILE FILES
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/message/flowmsg.h"
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/message/msg_registry.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/version.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto" TYPE FILE FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/version.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/session/xsession_info.h;/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/session/xsession.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/session" TYPE FILE FILES
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/session/xsession_info.h"
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/session/xsession.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/msg_type/control_message.h;/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/msg_type/smart_legible_message.h;/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/msg_type/statistics_message.h;/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/msg_type/vio_message.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/msg_type" TYPE FILE FILES
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/msg_type/control_message.h"
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/msg_type/smart_legible_message.h"
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/msg_type/statistics_message.h"
    "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/msg_type/vio_message.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output//xproto/include/xstream")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output//xproto/include" TYPE DIRECTORY FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xstream")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output//xproto/include/xproto/msg_type//protobuf")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output//xproto/include/xproto/msg_type/" TYPE DIRECTORY FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/msg_type/protobuf")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto/xproto_world.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/include/xproto" TYPE FILE FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/include/xproto/xproto_world.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug" TYPE SHARED_LIBRARY FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/libxproto.so")
  if(EXISTS "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so"
         OLD_RPATH "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/third_party/x86/centos/hobotlog/lib:/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/third_party/x86/centos/gtest/lib:/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/third_party/x86/centos/protobuf/lib:/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/third_party/x86/centos/zeroMQ/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/output/xproto/centos/shared_lib/debug/libxproto.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/test/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/tutorials/stage1_hello_world/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/tutorials/stage2_max_queue_size/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/tutorials/stage3_timeout_alarm/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/tutorials/stage4_ipc_subscriber/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xproto/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
