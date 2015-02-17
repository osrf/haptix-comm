set(TEST_TYPE "INTEGRATION")

set(tests
  twoProcesses.cc
)

set_property(
  SOURCE twoProcesses.cc
  PROPERTY COMPILE_DEFINITIONS
  BUILD_DIR="${CMAKE_BINARY_DIR}"
)

include_directories(${CMAKE_BINARY_DIR})

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(${tests})

# The twoProcesses test needs to run the hx_responser tool
ign_add_executable(hx_responser_test hx_responser.cc)
target_link_libraries(hx_responser_test
  ${PROJECT_NAME_LOWER} ${ZeroMQ_LIBRARIES})
