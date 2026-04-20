# KneeExo - Knee Exoskeleton Firmware
# Target: ESP32-S3-N16R8
# IDF version: v5.5.4

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(knee_exo)
