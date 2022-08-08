cmake_minimum_required(VERSION 3.10)

project(5ARM_Test)
set(CMAKE_CXX_STANDARD 11)
set(NAME test)

file(GLOB_RECURSE SOURCES "main.cpp")

add_subdirectory(Logi_Controller)


# add_executable(${PROJECT_NAME} ${SOURCES})
# target_link_libraries(${PROJECT_NAME} PUBLIC Logi_Contorller)

# target_include_directories(${PROJECT_NAME} PUBLIC
#                             "${CMAKE_BINARY_DIR}"
#                             "${PROJECT_SOURCE_DIR}/Logi_Controller")

add_executable(${NAME} ${SOURCES})
target_link_libraries(${NAME} PUBLIC Logi_Contorller)

target_include_directories(${NAME} PUBLIC
                            "${PROJECT_SOURCE_DIR}/Logi_Controller")