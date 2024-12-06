cmake_minimum_required(VERSION 3.22)

project(klibrary LANGUAGES CXX)

add_subdirectory(tutrcos)
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  include
)

target_link_libraries(${PROJECT_NAME} INTERFACE tutrcos)
