cmake_minimum_required (VERSION 3.18.0)

project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories(../EIGEN3_INCLUDE_DIR)

add_executable (Transformation main.cpp)

