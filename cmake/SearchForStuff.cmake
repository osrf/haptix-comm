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
find_package(ignition-transport QUIET REQUIRED)
if (NOT ignition-transport_FOUND)
  BUILD_ERROR ("Missing: Ignition Transport (libignition-transport-dev)")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

########################################
# Find boost: needed for the protobuf generator

include(FindBoost)
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)
  BUILD_ERROR ("Boost not found")
endif()

# Boost is needed not only in the haptixmsgs_out target but also during
# .pb.h compilation and other target including this header (like haptix lib). 
# This is the reason not to use target_include_directories and expand the
# include to all the project scope
include_directories(${Boost_INCLUDE_DIRS})

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

#################################################
# Find octave support (mkoctfile)
#
include (${project_cmake_dir}/FindOctaveOctFile.cmake)

#################################################
# Find MATLAB support (mex files)
#
include (${project_cmake_dir}/BuildMex.cmake)

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

