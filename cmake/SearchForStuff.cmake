include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

########################################
# Need to use _DEBUG postfix variables on windows to match the library
IF ((WIN32) AND (CMAKE_BUILD_TYPE MATCHES Debug))
  message(STATUS " - Using debug protobuf libraries on Windows")
  set(PROTOBUF_LIBRARY ${PROTOBUF_LIBRARY_DEBUG} CACHE FILEPATH "" FORCE)
  set(PROTOBUF_PROTOC_LIBRARY ${PROTOBUF_PROTOC_LIBRARY_DEBUG} CACHE FILEPATH "" FORCE)
endif()

########################################
# Find the Ignition_Transport library
find_package(ignition-transport0 QUIET REQUIRED)
if (NOT ignition-transport0_FOUND)
  BUILD_ERROR ("Missing: Ignition Transport (libignition-transport-dev)")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

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

