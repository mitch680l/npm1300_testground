# Install script for directory: C:/ncs/v3.0.2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/nrf91

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/ncs/toolchains/0b393f9e1b/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/platform/target/nrf91/nrf_common/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91/nrfx_config_nrf91.h;C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91/CMakeLists.txt;C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91/config.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91" TYPE FILE MESSAGE_NEVER FILES
    "C:/ncs/v3.0.2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/nrf91/nrfx_config_nrf91.h"
    "C:/ncs/v3.0.2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/nrf91/ns/CMakeLists.txt"
    "C:/ncs/v3.0.2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/nrf91/config.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91/tests;C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91/partition")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "C:/kes_new_features/npm1300_testground/build/npm1300_testground/tfm/api_ns/platform/common/nrf91" TYPE DIRECTORY MESSAGE_NEVER FILES
    "C:/ncs/v3.0.2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/nrf91/tests"
    "C:/ncs/v3.0.2/modules/tee/tf-m/trusted-firmware-m/platform/ext/target/nordic_nrf/common/nrf91/partition"
    )
endif()

