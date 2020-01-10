cmake_minimum_required(VERSION 3.5.0)
project(vo)
set(CMAKE_CXX_STANDARD 17)

find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(src/core VO_CORE_SRCS)
add_library(${PROJECT_NAME}_core ${VO_CORE_SRCS})


add_subdirectory(test)




