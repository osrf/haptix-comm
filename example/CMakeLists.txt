cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

########################################
# Find the haptix-comm library
find_package(haptix-comm QUIET REQUIRED)
if (NOT haptix-comm_FOUND)
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

#################################################
# Find ZeroMQ.
include (${PROJECT_SOURCE_DIR}/cmake/FindZeroMQ.cmake)

if (NOT ZeroMQ_FOUND)
  BUILD_ERROR ("zmq not found, Please install zmq")
else ()
  include_directories(${ZeroMQ_INCLUDE_DIRS})
  link_directories(${ZeroMQ_LIBRARY_DIRS})
endif ()

#################################################
# Find cppzeromq header (shipped together with zeromq in debian/ubuntu but
# different upstream projects and tarballs)
#
# Provide the PATH using CPPZMQ_HEADER_PATH
#
find_path(cppzmq_INCLUDE_DIRS
          zmq.hpp
	  PATHS
	   ${zmq_INCLUDE_DIRS}
	   ${CPPZMQ_HEADER_PATH})

if (NOT cppzmq_INCLUDE_DIRS)
  message(STATUS "cppzmq header file was not found")
  BUILD_ERROR("cppzmq header file was not found")
else()
  message(STATUS "cppzmq file - found")
  include_directories(${cppzmq_INCLUDE_DIRS})
endif()

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
########################################

if(WIN32)
  set(EXTRA_LIBRARIES ws2_32 Iphlpapi)
else()
  set(EXTRA_LIBRARIES m)
endif()

########################################
add_executable(requester hx_requester.c)
target_link_libraries(requester ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(responser hx_responser.cc)
target_link_libraries(responser ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(controller hx_controller.c)
target_link_libraries(controller ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})
