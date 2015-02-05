# BuildMex.cmake 
# 
# Needs: 
# - MATLAB_ROOT pointing to MATLAB/R20XXZ/
#

if(NOT MATLAB_MEX_PATH)
  find_program( MATLAB_MEX_PATH mex
    HINTS ${MATLAB_ROOT}/bin
    PATHS ${MATLAB_ROOT}/bin
    DOC "The mex program path")
endif()

if(NOT MATLAB_MEX_PATH)
  message(STATUS "MATLAB mex compiler not found - no mex file generation")
  BUILD_WARNING("MATLAB mex compiler not found - no mex file generation")
else()
  message(STATUS "MATLAB mex compiler found")
  find_path(MATLAB_INCLUDE_DIR
     "mex.h"
     ${MATLAB_ROOT}/extern/include)
endif()

macro (build_mex_file)
  # CMake 2.8.12 & earlier apparently don't define the
  # Mex script path, so find it.
   execute_process(COMMAND ${MATLAB_MEX_PATH} ${SOURCE_FILE} -I${CMAKE_BINARY_DIR} -L${CMAKE_BINARY_DIR}/src -L${CMAKE_BINARY_DIR}/msg -lhaptix-comm  -lhaptix-mgs -L${PROTOBUF_SRC_ROOT_FOLDER}/include -l${PROTOBUF_LIBRARY} -L${ZeroMQ_libzmq_LIBDIR} -l${ZeroMQ_LIBRARIES} -I${ignition-transport_DIR} -L${ignition-transport_DIR} -lignition-transport -lws2_32 -lIphlpapi
    OUTPUT_VARIABLE mexOut
    ERROR_VARIABLE mexErr)
endmacro()
