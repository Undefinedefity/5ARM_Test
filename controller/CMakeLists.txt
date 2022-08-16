cmake_minimum_required(VERSION 3.5)
# project(motor_test)

include_directories("./include")

add_library(controller
            "src/uart.cpp"
            "include/armMsg.hpp"
            "include/uart.hpp")

add_compile_options(-std=c++11)

# file(GLOB_RECURSE sources_arm "mainXiaotianArm.cpp" "math_ops.cpp" "uart.cpp")
# file(GLOB_RECURSE sources "main.cpp" "math_ops.cpp" "uart.cpp")

# add_executable(${PROJECT_NAME}_Arm ${sources_arm})
# target_link_libraries(${PROJECT_NAME}_Arm rt pthread)

# add_executable(${PROJECT_NAME} ${sources})
# target_link_libraries(${PROJECT_NAME} rt pthread)