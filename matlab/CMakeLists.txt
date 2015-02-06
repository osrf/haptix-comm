set (mex_srcs 
     hx_getdeviceinfo.c
     hx_update.c)

#
# Octave generation if support found
#
if (OCTAVE_SUPPORT_FOUND)

  set (OCTAVE_OUTPUT_FILE_SET)

  foreach(mex_src ${mex_srcs})
    get_filename_component(OCTAVE_NAME ${mex_src} NAME_WE)
    set(OCTAVE_NAME "${OCTAVE_NAME}.mex")
    set(OCTAVE_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${OCTAVE_NAME}")
    set(OCTAVE_OUTPUT_FILE_SET ${OCTAVE_OUTPUT_FILE_SET} ${OCTAVE_OUTPUT_FILE})
    
    add_custom_command(
      OUTPUT ${OCTAVE_OUTPUT_FILE}
      COMMAND ${MKOCTFILE} --mex -I${CMAKE_SOURCE_DIR}/include -l${PROJECT_NAME_LOWER} -L${CMAKE_BINARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/${mex_src}
      DEPENDS ${mex_srcs} ${PROJECT_NAME_LOWER}
      COMMENT "Generating Octave mex file ${OCTAVE_NAME}"
      VERBATIM)
  endforeach()

  install(FILES ${OCTAVE_OUTPUT_FILE_SET}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/octave)

  add_custom_target(mex_generation ALL DEPENDS ${OCTAVE_OUTPUT_FILE_SET})
endif()

#
# MATLAB generation if support found
# 
# Only tested on Windows. It could work on Linux but there is
# no guarantee or test done.
#
if (WIN32 AND MATLAB_FOUND)
  set (MATLAB_OUTPUT_FILE_SET)

  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(MEX_EXTENSION ".mexw32")
  else()
    set(MEX_EXTENSION ".mexw64")
  endif()

  foreach(mex_src ${mex_srcs})
    get_filename_component(MEX_NAME ${mex_src} NAME_WE)
    set(MEX_BINARY_NAME "${MEX_NAME}.${MEX_EXTENSION}")
    set(MEX_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${MEX_BINARY_NAME}")
    set(MEX_OUTPUT_FILE_SET ${MEX_OUTPUT_FILE_SET} ${MEX_OUTPUT_FILE})

    add_custom_command(
      OUTPUT ${MEX_OUTPUT_FILE}
      COMMAND ${MATLAB_MEX_PATH} ${SOURCE_FILE} -I${CMAKE_BINARY_DIR} -L${CMAKE_BINARY_DIR}/src -L${CMAKE_BINARY_DIR}/msg -lhaptix-comm  -lhaptix-mgs -L${PROTOBUF_SRC_ROOT_FOLDER}/include -l${PROTOBUF_LIBRARY} -L${ZeroMQ_LIBRARY_DIRS} -l${ZeroMQ_LIBRARIES} -I${ignition-transport_DIR} -L${ignition-transport_DIR} -lignition-transport -lws2_32 -lIphlpapi
      DEPENDS ${mex_srcs} ${PROJECT_NAME_LOWER} haptix-msgs
      COMMENT "Generating MATLAB mex file ${MEX_NAME}"
      VERBATIM)
  endforeach()

  install(FILES ${MEX_OUTPUT_FILE_SET}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/mex)

  add_custom_target(matlab_generation ALL DEPENDS ${MEX_OUTPUT_FILE_SET})
endif()
