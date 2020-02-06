aux_source_directory(core TEST_CORE_SRCS)
add_executable(test_core ${TEST_CORE_SRCS})
target_link_libraries(test_core ${PROJECT_NAME}_core ${GTEST_BOTH_LIBRARIES} pthread)

#aux_source_directory(utils TEST_UTILS_SRCS)
#add_executable(test_utils ${TEST_UTILS_SRCS})
#target_link_libraries(test_utils ${PROJECT_NAME}_utils ${GTEST_BOTH_LIBRARIES} pthread)
#
#aux_source_directory(io TEST_IO_SRCS)
#add_executable(test_io ${TEST_IO_SRCS})
#target_link_libraries(test_io ${PROJECT_NAME}_io ${GTEST_BOTH_LIBRARIES} pthread)

#add_test(test_core  test_core)
#add_test(test_utils test_utils)
#add_test(test_io    test_io)

add_executable(test_vo apps/test_vo.cpp)
target_link_libraries(test_vo  ${PROJECT_NAME}_core ${PROJECT_NAME}_viewer ${PROJECT_NAME}_utils ${PROJECT_NAME}_io)