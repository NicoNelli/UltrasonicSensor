cmake_minimum_required(VERSION 2.6)
project(UltrasonicSensor CXX)
add_definitions(-std=c++11)

# Set build directory
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)


find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

set(M2M_INCLUDE "$ENV{M2M_INCLUDE}")
if(M2M_INCLUDE)            
 include_directories(${M2M_INCLUDE})
else()
 message(STATUS "INCLUDE at ${M2M_INCLUDE}")
 message(FATAL_ERROR "Undefined M2M_INCLUDE env var. HINT: export M2M_INCLUDE=/path/to/mocap2mav/include")
endif()

include_directories(include)

set(SOURCE_FILES  src/UltrasonicSensor.cpp include/UltrasonicSensor.h )
add_library(UltrasonicSensor ${SOURCE_FILES})

find_library(wiringPi_LIB wiringPi)
target_link_libraries(UltrasonicSensor ${wiringPi_LIB})

add_executable(UltrasonicSensorExe src/main.cpp)

target_link_libraries(UltrasonicSensorExe UltrasonicSensor)
target_link_libraries(UltrasonicSensorExe ${LCM_LIBRARIES})

