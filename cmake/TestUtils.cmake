#################################################
macro (ign_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file})

    add_dependencies(${BINARY_NAME}
      ${PROJECT_NAME_LOWER}
      gtest gtest_main
      )

    if (UNIX)
      target_link_libraries(${BINARY_NAME}
        ${PROJECT_NAME_LOWER}
        libgtest.a
        libgtest_main.a
        pthread
        ${PROTOBUF_LIBRARY}
        ${ZeroMQ_LIBRARIES}
      )
    elseif(WIN32)
      target_link_libraries(${BINARY_NAME}
        ${PROJECT_NAME_LOWER}
        gtest
        gtest_main
        ${ZeroMQ_LIBRARIES}
        ${PROJECT_MSGS_NAME}
      )
      if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_link_libraries(${BINARY_NAME} ${PROTOBUF_LIBRARY_DEBUG})
      else()
        target_link_libraries(${BINARY_NAME} ${PROTOBUF_LIBRARY})
      endif()
    endif()

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	--gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} python ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
  endforeach()
endmacro()
