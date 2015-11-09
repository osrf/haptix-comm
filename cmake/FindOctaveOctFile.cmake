find_program(MKOCTFILE mkoctfile)

if (MKOCTFILE)
    set(OCTAVE_SUPPORT_FOUND TRUE)
    message(STATUS "Octave mex compiler found.  "
                   "Generating mex files for octave"
                   " with mkoctfile at ${MKOCTFILE}")
else()
    set(OCTAVE_SUPPORT_FOUND FALSE)
    BUILD_WARNING("Octave mex file compiler not found, did you forget to install liboctave-dev?")
endif()
