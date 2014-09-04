set(TEST_TYPE "INTEGRATION")

set(tests
  twoProcesses.cc
)

include_directories(${CMAKE_BINARY_DIR})

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(${tests})
