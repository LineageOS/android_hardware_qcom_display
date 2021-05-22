project(NOISEPLUGIN-TEST)
cmake_minimum_required(VERSION 3.12)
add_definitions("-DNOISE_PLUGIN_STAND_ALONE")
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")

set(SDM_PATH $ENV{ANDROID_BUILD_TOP}/hardware/qcom/display)

include_directories (${SDM_PATH}/sdm/include/
                     ${SDM_PATH}/libdebug/
                     ${SDM_PATH}/sdm/include/private/)

add_library(noiseplugin SHARED
            ${SDM_PATH}/sdm/libs/core/noise_plugin_intf_impl.cpp
            ${SDM_PATH}/libdebug/debug_handler.cpp)

add_executable(noiseplugin_test
               ${SDM_PATH}/sdm/libs/core/noise_plugin_unit_test.cpp)

target_link_libraries(noiseplugin_test
                      noiseplugin)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_SKIP_BUILD_RPATH true)
