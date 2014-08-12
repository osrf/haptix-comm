include (${project_cmake_dir}/Utils.cmake)

set (sources
  AplControl.pb.cc
  Comm.cc
)

set (gtest_sources
  Comm_TEST.cc
)

# Generate the protobuf code.
# PROTOBUF_GENERATE_C(PROTO_SRCS PROTO_HDRS Arm.proto)

ign_build_tests(${gtest_sources})
ign_add_library(${PROJECT_NAME_LOWER} ${sources})
target_link_libraries(${PROJECT_NAME_LOWER} ignition-transport)
ign_install_library(${PROJECT_NAME_LOWER})
