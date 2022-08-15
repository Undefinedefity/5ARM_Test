cmake_minimum_required(VERSION 3.10)

project(Logi_Controller_Test)
set(CMAKE_CXX_STANDARD 11)

include_directories("./include")

file(GLOB_RECURSE SOURCES "src/*.cpp")

message(STATUS "SOURCES is " ${SOURCES})

add_library(Logi_Contorller ${SOURCES})

# add_executable(${PROJECT_NAME}_A ${SOURCES} "Test_A.cpp")

# add_executable(${PROJECT_NAME}_B ${SOURCES} "Test_B.cpp")

add_executable(${PROJECT_NAME}_C ${SOURCES} "Test_C.cpp")