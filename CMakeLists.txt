cmake_minimum_required(VERSION 3.8)
project(spot_micro_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(spot_micro_kinematics
  kinematics/spot_micro_kinematics.cc 
  kinematics/spot_micro_matrix.cc
)

target_include_directories(spot_micro_kinematics
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include 
)
