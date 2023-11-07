cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)

# Set and enforce C++-11 flags
set( CMAKE_CXX_STANDARD_REQUIRED TRUE )
set( CMAKE_CXX_STANDARD 11 )

project(BoolOperation2D)

add_executable(test_demo main.cpp)
add_executable(test_grammar test.cpp)
add_library(bool_ops SHARED "src/bool.cpp" "src/bool2.cpp")

target_include_directories(bool_ops PUBLIC "./include")

target_link_libraries(test_demo bool_ops)