# Install script for directory: /home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream

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
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/include//xstream")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/include/" TYPE DIRECTORY FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/include/xstream")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug" TYPE SHARED_LIBRARY FILES "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/libxstream.so")
  if(EXISTS "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so"
         OLD_RPATH "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/third_party/x86/centos/jsoncpp/lib:/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/third_party/x86/centos/hobotlog/lib:/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/third_party/x86/centos/gtest/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/output/xstream/centos/shared_lib/debug/libxstream.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/test/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/benchmark/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage1_hello_world/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage2_multithread/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage3_update_parameter/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage4_multisource/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage5_timeout_alarm/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage6_profile/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage7_node_output/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage8_sub_workflow/cmake_install.cmake")
  include("/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/tutorials/stage9_disable_method/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jenkins/workspace/atform_ai_express_common_1.1.10d/xstream/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
