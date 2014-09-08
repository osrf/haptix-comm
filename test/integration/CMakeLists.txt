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
