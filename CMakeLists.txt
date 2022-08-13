cmake_minimum_required(VERSION 3.8)
project(quadruped_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(quadruped_kinematics
  kinematics/quadruped_kinematics.cc 
  kinematics/quadruped_matrix.cc
)

target_include_directories(quadruped_kinematics
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include 
)
