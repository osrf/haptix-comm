set (mex_srcs 
     hx_getdeviceinfo.c
     hx_update.c)

set (OCTAVE_OUTPUT_FILE_SET)

foreach(mex_src ${mex_srcs})
    get_filename_component(OCTAVE_NAME ${mex_src} NAME_WE)
    set(OCTAVE_NAME "${OCTAVE_NAME}.mex")
    set(OCTAVE_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${OCTAVE_NAME}")
    set(OCTAVE_OUTPUT_FILE_SET ${OCTAVE_OUTPUT_FILE_SET} ${OCTAVE_OUTPUT_FILE})

    add_custom_command(
	OUTPUT ${OCTAVE_OUTPUT_FILE}
	COMMAND ${MKOCTFILE} --mex -I${CMAKE_SOURCE_DIR}/include -l${PROJECT_NAME_LOWER} -L${CMAKE_BINARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/${mex_src}
	COMMENT "Generating mex file ${OCTAVE_NAME}"
	VERBATIM)
endforeach()

install(FILES ${OCTAVE_OUTPUT_FILE_SET}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/octave)

add_custom_target(mex_generation DEPENDS ${OCTAVE_OUTPUT_FILE_SET})
