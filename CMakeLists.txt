# Used ONLY by the hybrid (framework = arduino, espidf) envs -- today
# esp32-s3-N16R8-pm. The arduino-only envs never read this file (their builder
# ignores CMake); it exists because a from-source ESP-IDF build requires the
# project to be a valid IDF project. sdkconfig.defaults at the repo root is
# picked up by PlatformIO's espidf builder automatically.
cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(opendisplay-firmware)
