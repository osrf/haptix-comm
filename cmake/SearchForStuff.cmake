include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

########################################
# The Google Protobuf library for message generation + serialization
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

# Need to use _DEBUG postfix variables on windows to match the library
IF ((WIN32) AND (CMAKE_BUILD_TYPE MATCHES Debug))
  message(STATUS " - Using debug protobuf libraries on Windows")
  set(PROTOBUF_LIBRARY ${PROTOBUF_LIBRARY_DEBUG} CACHE FILEPATH "" FORCE)
  set(PROTOBUF_PROTOC_LIBRARY ${PROTOBUF_PROTOC_LIBRARY_DEBUG} CACHE FILEPATH "" FORCE)
endif()

########################################
# Find the Ignition_Transport library
set(IGNITION_TRANSPORT_MAJOR_VERSION 1)
set(IGNITION_TRANSPORT_PACKAGE "ignition-transport${IGNITION_TRANSPORT_MAJOR_VERSION}")
find_package("${IGNITION_TRANSPORT_PACKAGE}" QUIET REQUIRED)
if (NOT ${IGNITION_TRANSPORT_PACKAGE}_FOUND)
  BUILD_ERROR ("Missing: Ignition Transport ${IGNITION_TRANSPORT_MAJOR_VERSION} (libignition-transport${IGNITION_TRANSPORT_MAJOR_VERSION}-dev)")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

#################################################
# Find ZeroMQ.
include (${project_cmake_dir}/FindZeroMQ.cmake)

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
find_path(cppzmq_INCLUDE_DIRS zmq.hpp
          PATHS ${zmq_INCLUDE_DIRS} ${CPPZMQ_HEADER_PATH})

if (NOT cppzmq_INCLUDE_DIRS)
  message(STATUS "cppzmq header file was not found")
  BUILD_ERROR("cppzmq header file was not found")
else()
  message(STATUS "cppzmq file - found")
  include_directories(${cppzmq_INCLUDE_DIRS})
endif()

#################################################
# Find octave support (mkoctfile)
#
include (${project_cmake_dir}/FindOctaveOctFile.cmake)

#################################################
# Find MATLAB support (mex files)
#
include (${project_cmake_dir}/FindMex.cmake)

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

