include (${project_cmake_dir}/Utils.cmake)

set (sources
  Comm.cc
)

set (gtest_sources
  Comm_TEST.cc
)

ign_build_tests(${gtest_sources})
ign_add_library(${PROJECT_NAME_LOWER} ${sources})
target_link_libraries(${PROJECT_NAME_LOWER} ignition-transport)
ign_install_library(${PROJECT_NAME_LOWER})
