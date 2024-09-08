# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ravelights-pixeldriver)

# esp-rainmaker dependency throws a string truncation warning that is treated as an error when the compiler
# optimization level is set to -O2 (optimize for performance).
idf_build_set_property(C_COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)