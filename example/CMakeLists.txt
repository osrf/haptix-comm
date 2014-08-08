cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Ignition_Transport library
find_package(comm QUIET REQUIRED)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMM_C_FLAGS}")

include_directories(${COMM_INCLUDE_DIRS}
  ${COMM_INCLUDE_DIRS})
link_directories(${COMM_LIBRARY_DIRS}
  ${COMM_LIBRARY_DIRS})

add_executable(requester_c requester_c.c)
target_link_libraries(requester_c ${COMM_LIBRARIES})
