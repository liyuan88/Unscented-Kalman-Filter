cmake_minimum_required(VERSION 3.6)
project(Unscented_Kalman_Filter)

set(DCMAKE_CXX_COMPILER "g++-5")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        src/main.cpp
        src/ukf.cpp
        src/tools.cpp)
add_executable(Unscented_Kalman_Filter ${SOURCE_FILES})

add_definitions(-std=c++0x)