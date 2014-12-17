set (wrappers
  hx_getdeviceinfo.c
  hx_update.c
)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

# Generate the mex files for Octave
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
  #find_program(MEX_GENERATOR mkoctfile)
  find_program(MEX_GENERATOR touch)
  if (EXISTS ${MEX_GENERATOR})
    foreach (file ${wrappers})
      get_filename_component(MEX_FILE ${file} NAME_WE)
      message("${MEX_FILE}.mex")
      add_custom_command(
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${MEX_FILE}.mex
        #COMMAND ${MEX_GENERATOR} -I${CMAKE_SOURCE_DIR}/include
        #  -l${PROJECT_NAME_LOWER} -L${CMAKE_BINARY_DIR}/src --mex ${file}
        COMMAND ${MEX_GENERATOR} ${CMAKE_CURRENT_SOURCE_DIR}/${MEX_FILE}.mex
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${MEX_FILE}.mex
        COMMENT "${Cyan}Generating ${MEX_FILE}.mex${ColourReset}"
      )
      add_custom_target(${MEX_FILE}_mex DEPENDS "${MEX_FILE}.mex" ${file})

      set_source_files_properties("${MEX_FILE}.mex" PROPERTIES GENERATED TRUE)

      add_dependencies(${MEX_FILE}_mex ALL)
    endforeach()
  endif(EXISTS ${MEX_GENERATOR})
endif()