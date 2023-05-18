cmake_minimum_required(VERSION 3.8.0)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 14)
project(avoid_obstacles)

option(REALSENSE "" ON)

find_package(Protobuf)

if(PROTOBUF_FOUND)
    message(STATUS "    version: ${Protobuf_VERSION}")
    message(STATUS "    libraries: ${PROTOBUF_LIBRARIES}")
    message(STATUS "    include path: ${PROTOBUF_INCLUDE_DIR}")
else()
    message(WARNING "Protobuf not found, onnx model convert tool won't be built")
endif()

if (REALSENSE)
    find_package(realsense2 REQUIRED)
endif()

#find_package( OpenCV 3.2 REQUIRED )
#set(OpenCV_DIR "/home/ash/Ash/repo/opencv/build")
set(CMAKE_PREFIX_PATH  "/home/ash/Ash/repo/opencv/build")
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "realsense2_LIBRARY = ${realsense2_LIBRARY}")
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}
        src/main.cpp
#        src/main1.cpp
#        src/downsample.cpp
#        include/tiny-profiler.h
)

target_link_libraries(
    ${PROJECT_NAME}
    ${DEPENDENCIES}
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
)

#target_include_directories(${TORCH_INCLUDE_DIRS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

####################################
####  Gtest ##################
####################################

#enable_testing()
#include(FetchContent)
#FetchContent_Declare(
#        googletest
#        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
#)
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#FetchContent_MakeAvailable(googletest)
#
#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

######## 1  #########
#[[add_executable(
        load_yaml_test
        test/load_yaml_test.cpp
)
include_directories(
#        ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(
        load_yaml_test
#        ${YAML_CPP_LIBRARIES}
        gtest
#        gtest_main
)
add_test( load_yaml_test load_yaml_test)]]