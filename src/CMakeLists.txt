include (${project_cmake_dir}/Utils.cmake)

set (sources
  haptix.cc
#  haptix_sim.cc
)

set (gtest_sources
#  haptix_TEST.cc
)

set_source_files_properties(${PROTO_SRC} PROPERTIES GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR})

#ign_build_tests(${gtest_sources})
if (UNIX)
   ign_add_library(${PROJECT_NAME_LOWER} ${sources} ${PROTO_SRC})
elseif(WIN32)
  # Probotuf compilation is recommended and configured for a STATIC .lib
  # Not mixing static and shared and just build ignition transport as STATIC
  # on Windows
  #ign_add_library(${PROJECT_NAME_LOWER} ${sources} ${PROTO_SRC})
  ign_add_hg library(${PROJECT_NAME_LOWER} ${sources} ${PROTO_SRC})
endif()
target_link_libraries(${PROJECT_NAME_LOWER}
                      ${ZeroMQ_LIBRARIES})
#                      ${IGNITION-TRANSPORT_LIBRARIES})
target_link_libraries(${PROJECT_NAME_LOWER} ${PROTOBUF_LIBRARY})
if(WIN32)
  target_link_libraries(${PROJECT_NAME_LOWER} ws2_32 Iphlpapi ${PROTOBUF_LIBRARY})
endif()
ign_install_library(${PROJECT_NAME_LOWER})

add_dependencies(${PROJECT_NAME_LOWER}
#  gtest gtest_main
  protobuf_compilation
)
