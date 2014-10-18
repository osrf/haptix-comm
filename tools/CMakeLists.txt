include_directories(
  ${PROTOBUF_INCLUDE_DIR}
)

add_executable(hx_responser hx_responser.cc)
include_directories(${CMAKE_BINARY_DIR})
target_link_libraries(hx_responser ${PROJECT_NAME_LOWER})
