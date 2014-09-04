cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

########################################
# Find the haptix_comm library
find_package(haptix_comm QUIET REQUIRED)
if (NOT haptix_comm_FOUND)
  BUILD_ERROR ("Missing: Haptix Comm (libhaptix-comm-dev)")
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${HAPTIX_COMM_CFLAGS}")
include_directories(${HAPTIX_COMM_INCLUDE_DIRS})
link_directories(${HAPTIX_COMM_LIBRARY_DIRS})

########################################
# Find the Ignition_Transport library
find_package(ignition-transport QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

########################################
# The Google Protobuf library
find_package(Protobuf REQUIRED)
if (NOT PROTOBUF_FOUND)
  BUILD_ERROR ("Missing: Google Protobuf (libprotobuf-dev)")
endif()
if (NOT PROTOBUF_PROTOC_EXECUTABLE)
  BUILD_ERROR ("Missing: Google Protobuf Compiler (protobuf-compiler)")
endif()
if (NOT PROTOBUF_PROTOC_LIBRARY)
  BUILD_ERROR ("Missing: Google Protobuf Compiler Library (libprotoc-dev)")
endif()

add_subdirectory(msg)

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES
                            GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR})

########################################
add_executable(requester requester.c)
target_link_libraries(requester ${HAPTIX_COMM_LIBRARIES})

add_executable(responser responser.cc ${PROTO_SRC})
target_link_libraries(responser ${IGNITION-TRANSPORT_LIBRARIES} protobuf)
add_dependencies(responser protobuf_compilation)
