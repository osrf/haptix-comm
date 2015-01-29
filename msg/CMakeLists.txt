set(msgs
  hxCommand.proto
  hxDevice.proto
  hxSensor.proto
  hxGrasp.proto
)

add_executable(haptixmsgs_out generator/HaptixGenerator.cc generator/haptix_generator.cc)
target_include_directories(haptixmsgs_out PUBLIC ${Boost_INCLUDE_DIRS})

target_link_libraries(haptixmsgs_out ${PROTOBUF_LIBRARY} ${PROTOBUF_PROTOC_LIBRARY})

if (UNIX)
  target_link_libraries(haptixmsgs_out pthread)
endif()

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(FIL ${msgs})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)

  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --plugin=protoc-gen-haptixmsgs=${CMAKE_CURRENT_BINARY_DIR}/haptixmsgs_out --cpp_out ${CMAKE_CURRENT_BINARY_DIR} --haptixmsgs_out ${CMAKE_CURRENT_BINARY_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL} haptixmsgs_out
    COMMENT "Running C++ protocol buffer compiler on ${FIL}"
    VERBATIM )
endforeach()

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

ign_add_library(haptix_msgs ${PROTO_SRCS})
ign_install_library(haptix_msgs)
add_dependencies(haptix_msgs haptixmsgs_out)

# Install the generated protobuf messages.
install(FILES ${PROTO_HDRS} ${PROTO_SRCS} DESTINATION ${INCLUDE_INSTALL_DIR}/comm/msg)
