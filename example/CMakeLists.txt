cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

########################################
# Find the haptix-comm library
find_package(haptix-comm QUIET REQUIRED)
if (NOT haptix-comm_FOUND)
  BUILD_ERROR ("Missing: Haptix Comm (libhaptix-comm-dev)")
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${HAPTIX-COMM_CFLAGS}")
include_directories(${HAPTIX-COMM_INCLUDE_DIRS})
link_directories(${HAPTIX-COMM_LIBRARY_DIRS})

########################################
# Find the Ignition_Transport library
find_package(ignition-transport0 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

########################################

if(WIN32)
  set(EXTRA_LIBRARIES ws2_32 Iphlpapi)
else()
  set(EXTRA_LIBRARIES m)
endif()

########################################
add_executable(requester hx_requester.c)
target_link_libraries(requester ${HAPTIX-COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(responser hx_responser.cc)
target_link_libraries(responser ${HAPTIX-COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(controller hx_controller.c)
target_link_libraries(controller ${HAPTIX-COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(joint_stress hx_joint_stress.c)
target_link_libraries(joint_stress ${HAPTIX-COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(joints_stress hx_joints_stress.c)
target_link_libraries(joints_stress ${HAPTIX-COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

add_executable(hxs_requester hxs_requester.c)
target_link_libraries(hxs_requester ${HAPTIX-COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARIES} ${EXTRA_LIBRARIES})

if (MSVC)
# Copy the ZMQ DLLs.
  add_custom_command(TARGET requester POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "../../../ZeroMQ 3.2.4/bin/libzmq-v120-mt-3_2_4.dll"
      $<TARGET_FILE_DIR:requester> VERBATIM)

  add_custom_command(TARGET requester POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "../../../ZeroMQ 3.2.4/bin/libzmq-v120-mt-gd-3_2_4.dll"
      $<TARGET_FILE_DIR:requester> VERBATIM)
endif()
