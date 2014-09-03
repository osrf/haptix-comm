include (${project_cmake_dir}/Utils.cmake)

set (sources
  comm.cc
  comm_sim.cc
)

set (gtest_sources
  comm_TEST.cc
)

set_source_files_properties(${PROTO_SRC} PROPERTIES GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR})

ign_build_tests(${gtest_sources})
ign_add_library(${PROJECT_NAME_LOWER} ${sources} ${PROTO_SRC})
target_link_libraries(${PROJECT_NAME_LOWER} ignition-transport)
ign_install_library(${PROJECT_NAME_LOWER})

add_dependencies(${PROJECT_NAME_LOWER}
  gtest gtest_main
  protobuf_compilation
)
