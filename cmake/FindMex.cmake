# FindMex.cmake 
# 
# Needs: 
# - 'mex' on the PATH; OR:
# - MATLAB_ROOT pointing to MATLAB/R20XXZ/
#
# It will look for mex[.bat] script and provide the following variables:
#    - MATLAB_MEX_PATH    : full path to MEX script
#    - MATLAB_FOUND       : 0/1 -> support not found / found
#    - MATLAB_INCLUDE_DIR : full path to extern/include. Mex headers.

# TODO: check whether the explicit .bat extension is required for find_program()
# to work.
if(WIN32)
  set(MEX_NAME mex.bat)
else()
  set(MEX_NAME mex)
endif()

if(NOT MATLAB_MEX_PATH)
  find_program( MATLAB_MEX_PATH ${MEX_NAME}
    HINTS ${MATLAB_ROOT}/bin
    PATHS ${MATLAB_ROOT}/bin
    DOC "The mex program path")
endif()

if(NOT MATLAB_MEX_PATH)
  set (MATLAB_FOUND 0)
  BUILD_WARNING("MATLAB mex compiler not found - no mex file generation")
else()

  if (NOT WIN32)
    # Check if pdfTeX was found, instead of matlab. A number > 0 will be
    # returned if pdfTeX was found.
    exec_program(${MATLAB_MEX_PATH} ARGS "-v | grep pdfTeX | wc -l"
      OUTPUT_VARIABLE PDF_TEX_TEST)
  else()
    # If on windows, set PDF_TEX_TEST to zero
    set (PDF_TEX_TEST 0)
  endif()

  # If pdfTeX test failed, then matlab was found.
  if (NOT ${PDF_TEX_TEST})
    set (MATLAB_FOUND 1)
    message(STATUS "MATLAB mex compiler found")
    # TODO: check that MATLAB_INCLUDE_DIR is actually needed anywhere
    find_path(MATLAB_INCLUDE_DIR
       "mex.h"
       ${MATLAB_ROOT}/extern/include)
  else ()
    BUILD_WARNING("MATLAB mex compiler not found - no mex file generation")
  endif()

endif()
