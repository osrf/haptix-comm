cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Ignition_Transport library
find_package(comm QUIET REQUIRED)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMM_C_FLAGS}")

# Find the Ignition_Msgs library
find_package(ignition-msgs QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-MSGS_CXX_FLAGS}")

include_directories(${COMM_INCLUDE_DIRS}
  ${COMM_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS})
link_directories(${COMM_LIBRARY_DIRS}
  ${COMM_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS})

add_executable(requester requester.c)
target_link_libraries(requester haptix_comm ${IGNITION-MSGS_LIBRARIES})

add_executable(responser responser.c)
target_link_libraries(responser haptix_comm ${IGNITION-MSGS_LIBRARIES})
