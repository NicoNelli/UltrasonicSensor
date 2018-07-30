cmake_minimum_required(VERSION 2.6)
project(UltrasonicSensor CXX)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})


set(SOURCE_FILES  UltrasonicSensor.cpp UltrasonicSensor.h )
add_library(UltrasonicSensor ${SOURCE_FILES})

find_library(wiringPi_LIB wiringPi)
target_link_libraries(UltrasonicSensor ${wiringPi_LIB})

add_executable(prova main.cpp)
target_link_libraries(prova UltrasonicSensor)
