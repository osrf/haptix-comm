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
########################################

find_library(RTMIDI rtmidi)
find_library(YAMLCPP yaml-cpp)
find_library(NCURSES ncurses)
find_package(ignition-math)
find_package(gazebo)

########################################
add_executable(requester hx_requester.c)
target_link_libraries(requester ${HAPTIX_COMM_LIBRARIES})

add_executable(responser hx_responser.cc)
target_link_libraries(responser ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

if(NOT YAMLCPP-NOTFOUND)

  if(NOT RTMIDI-NOTFOUND)
    add_executable(korg_finger_teleop korg_finger_teleop.cc sliders/sliders.cpp)
    target_link_libraries(korg_finger_teleop ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${RTMIDI} ${YAMLCPP})

    add_executable(korg_teleop korg_teleop.cc sliders/sliders.cpp)
    target_link_libraries(korg_teleop ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${RTMIDI} ${YAMLCPP})
  endif()

  # alsamidi version
  add_executable(korg_teleop_amidi korg_teleop_amidi.cc sliders/sliders_amidi.cpp)
  target_link_libraries(korg_teleop_amidi ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} /usr/lib/x86_64-linux-gnu/libasound.so)

  if(NOT NCURSES-NOTFOUND)
    include_directories(${IGNITION-MATH_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
    link_directories(${IGNITION-MATH_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})

    add_executable(keyboard_teleop keyboard_teleop.cc)
    target_link_libraries(keyboard_teleop ${HAPTIX_COMM_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${YAMLCPP} ${NCURSES} ${GAZEBO_LIBRARIES})

  endif()

endif()
