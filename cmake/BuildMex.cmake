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
  set (MATLAB_FOUND 0)
  BUILD_WARNING("MATLAB mex compiler not found - no mex file generation")
else()
  set (MATLAB_FOUND 1)
  message(STATUS "MATLAB mex compiler found")
  find_path(MATLAB_INCLUDE_DIR
     "mex.h"
     ${MATLAB_ROOT}/extern/include)
endif()
