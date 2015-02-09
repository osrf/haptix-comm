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

  # TODO: save check to be sure that variables needed for the hack
  # below is working
  if (NOT defined ignition-transport_DIR)
     BUILD_ERROR("MATLAB support found but there is no ignition-transport_DIR variable")
  endif()

  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(MEX_EXTENSION "mexw64")
  else()
    set(MEX_EXTENSION "mexw32")
  endif()

  foreach(mex_src ${mex_srcs})
    get_filename_component(MEX_NAME ${mex_src} NAME_WE)
    set(MEX_BINARY_NAME "${MEX_NAME}.${MEX_EXTENSION}")
    set(MEX_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${MEX_BINARY_NAME}")
    set(MEX_OUTPUT_FILE_SET ${MEX_OUTPUT_FILE_SET} ${MEX_OUTPUT_FILE})

    # TODO: IGNITION cmake variables can not be directly used, they stored 
    # several paths separated by an space which will break this whole
    # mex hack.

    # ignition-transport_DIR points to the cmake module location
    set (mex_ignition_transport_root ${ignition-transport_DIR}/../../..)
    set (mex_ignition_transport_include ${mex_ignition_transport_root}/include)
    set (mex_ignition_transport_lib ${mex_ignition_transport_root}/lib)

    add_custom_command(
      OUTPUT ${MEX_OUTPUT_FILE}
      COMMAND ${MATLAB_MEX_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/${mex_src} ${PROTOBUF_LIBRARY} ${ZeroMQ_LIBRARY_RELEASE} -I${CMAKE_SOURCE_DIR}/include -L${CMAKE_BINARY_DIR}/src -L${CMAKE_BINARY_DIR}/msg -lhaptix-comm -lhaptix-msgs -I${mex_ignition_transport_include} -L${mex_ignition_transport_lib} -lignition-transport -lws2_32 -lIphlpapi
      DEPENDS ${mex_srcs} ${PROJECT_NAME_LOWER} haptix-msgs
      COMMENT "Generating MATLAB mex file ${MEX_NAME}"
      VERBATIM)
  endforeach()

  install(FILES ${MEX_OUTPUT_FILE_SET}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/mex)

  add_custom_target(matlab_generation ALL DEPENDS ${MEX_OUTPUT_FILE_SET})
endif()
