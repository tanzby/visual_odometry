cmake_minimum_required(VERSION 3.5.0)
project(vo)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# gtest
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# yaml-cpp
find_package(yaml-cpp 0.6 REQUIRED)

# g2o
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# pangolin
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# dBow3
find_package(DBoW3 REQUIRED)
include_directories(${DBoW3_INCLUDE_DIRS})

include_directories(src)
set(PROJECT_COMMON_LIB
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_TYPES_SBA}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${CSPARSE_LIBRARY}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${YAML_CPP_LIBRARIES})

aux_source_directory(src/core VO_CORE_SRCS)
add_library(${PROJECT_NAME}_core ${VO_CORE_SRCS})
target_link_libraries(${PROJECT_NAME}_core ${PROJECT_COMMON_LIB})

aux_source_directory(src/viewer VO_VIEWER_SRCS)
add_library(${PROJECT_NAME}_viewer ${VO_VIEWER_SRCS})
target_link_libraries(${PROJECT_NAME}_viewer ${PROJECT_COMMON_LIB})

aux_source_directory(src/utils VO_UTILS_SRCS)
add_library(${PROJECT_NAME}_utils ${VO_UTILS_SRCS})
target_link_libraries(${PROJECT_NAME}_utils ${PROJECT_COMMON_LIB})

aux_source_directory(src/io VO_IO_SRCS)
add_library(${PROJECT_NAME}_io ${VO_IO_SRCS})
target_link_libraries(${PROJECT_NAME}_io ${PROJECT_COMMON_LIB} ${PROJECT_NAME}_core)

add_subdirectory(test)




