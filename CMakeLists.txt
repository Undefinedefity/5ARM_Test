cmake_minimum_required(VERSION 3.10)
project(5ARM_Test)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(Logi_Controller)
add_subdirectory(thirdparty)
add_subdirectory(controller)

include_directories("thirdparty/DynamixelSDK/include/dynamixel_sdk")
include_directories("Logi_Controller/include")
include_directories("controller/include")

add_executable(main
               "main.cpp"
               "main.h")

target_link_libraries(main PUBLIC
                      controller
                      Logi_Contorller
                      DynamixelSDK
                      rt 
                      pthread
                      )