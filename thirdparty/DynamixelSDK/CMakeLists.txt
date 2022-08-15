cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)

include_directories("./include/dynamixel_sdk")
include_directories(${CMAKE_SOURCE_DIR}/Logi_Controller/include)

file(GLOB_RECURSE SOURCES "src/*.h" "src/dynamixel_sdk/*.cpp")

add_library(DynamixelSDK ${SOURCES})

add_executable(Servo_Test_1 ${SOURCES} "test1.cpp")
add_executable(Servo_Control ${SOURCES} "servo_control.cpp")
target_link_libraries(Servo_Control PUBLIC Logi_Contorller)