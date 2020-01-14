include(ExternalProject)
ExternalProject_Add(
    google_gtest
    URL https://github.com/google/googletest/archive/release-1.8.0.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/third/gtest
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/third/gtest/build -DBUILD_SHARED_LIBS=OFF
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/third/gtest/build/include ../src/)
link_directories("${CMAKE_CURRENT_BINARY_DIR}/third/gtest/build/lib")

add_executable(test_core test_core.cpp test_lie.cpp)
target_link_libraries(test_core PRIVATE ${PROJECT_NAME}_core gtest pthread)
add_dependencies(test_core google_gtest)

add_test(test_core test_core)