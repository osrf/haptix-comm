set (mex_srcs
     hxgz.c)

set (matlab_files
  hx_close.m
  hx_connect.m
  hx_matlab_controller.m
  hx_read_sensors.m
  hx_robot_info.m
  hxs_add_model.m
  hxs_angular_velocity.m
  hxs_apply_force.m
  hxs_apply_torque.m
  hxs_apply_wrench.m
  hxs_camera_transform.m
  hxs_contacts.m
  hxs_example.m
  hxs_is_logging.m
  hxs_linear_velocity.m
  hxs_model_collide_mode.m
  hxs_model_color.m
  hxs_model_gravity_mode.m
  hxs_model_joint_state.m
  hxs_model_transform.m
  hxs_remove_model.m
  hxs_reset.m
  hxs_set_angular_velocity.m
  hxs_set_camera_transform.m
  hxs_set_linear_velocity.m
  hxs_set_model_collide_mode.m
  hxs_set_model_color.m
  hxs_set_model_gravity_mode.m
  hxs_set_model_joint_state.m
  hxs_set_model_transform.m
  hxs_sim_info.m
  hxs_start_logging.m
  hxs_stop_logging.m
  hx_update.m)

#
# Octave generation if support found
#
if (OCTAVE_SUPPORT_FOUND)

  set (OCTAVE_OUTPUT_FILE_SET)

  foreach(mex_src ${mex_srcs})
    get_filename_component(OCTAVE_NAME ${mex_src} NAME_WE)
    set(OCTAVE_NAME "${OCTAVE_NAME}.mex")
    set(OCTAVE_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/octave/${OCTAVE_NAME}")
    set(OCTAVE_OUTPUT_FILE_SET ${OCTAVE_OUTPUT_FILE_SET} ${OCTAVE_OUTPUT_FILE})

    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/octave)
    add_custom_command(
      OUTPUT ${OCTAVE_OUTPUT_FILE}
      COMMAND ${MKOCTFILE} --mex -o ${OCTAVE_OUTPUT_FILE} -I${CMAKE_SOURCE_DIR}/include -l${PROJECT_NAME_LOWER} -L${CMAKE_BINARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/${mex_src}
      DEPENDS ${mex_srcs} ${PROJECT_NAME_LOWER}
      COMMENT "Generating Octave mex file ${OCTAVE_NAME}"
      VERBATIM)
  endforeach()

  install(FILES ${OCTAVE_OUTPUT_FILE_SET}
              ${matlab_files}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/haptix-comm/octave)

  add_custom_target(octave_generation ALL DEPENDS ${OCTAVE_OUTPUT_FILE_SET})
endif()

#
# MATLAB generation if support found
#
# Only tested on Windows. It could work on Linux but there is
# no guarantee or test done.
#
if (MATLAB_FOUND)
  set (MEX_OUTPUT_FILE_SET)

  # TODO: save check to be sure that variables needed for the hack
  # below is working
  if (NOT DEFINED ignition-transport0_DIR)
     BUILD_ERROR("MATLAB support found but there is no ignition-transport0_DIR variable")
  endif()

  if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(MEX_EXTENSION "mexw64")
    else()
      set(MEX_EXTENSION "mexw32")
    endif()
  else()
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(MEX_EXTENSION "mexa64")
    else()
      set(MEX_EXTENSION "mexa32")
    endif()
  endif()

  foreach(mex_src ${mex_srcs})
    get_filename_component(MEX_NAME ${mex_src} NAME_WE)
    set(MEX_BINARY_NAME "${MEX_NAME}.${MEX_EXTENSION}")
    set(MEX_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/matlab/${MEX_BINARY_NAME}")
    set(MEX_OUTPUT_FILE_SET ${MEX_OUTPUT_FILE_SET} ${MEX_OUTPUT_FILE})

    # TODO: IGNITION cmake variables can not be directly used, they stored
    # several paths separated by an space which will break this whole
    # mex hack.

    # ignition-transport0_DIR points to the cmake module location
    set (mex_ignition_transport_root ${ignition-transport0_DIR}/../../..)
    set (mex_ignition_transport_include ${mex_ignition_transport_root}/include)
    set (mex_ignition_transport_lib ${mex_ignition_transport_root}/lib)

    set(MEX_FLAGS -outdir ${CMAKE_CURRENT_BINARY_DIR}/matlab ${PROTOBUF_LIBRARY}
        ${ZeroMQ_LIBRARY_RELEASE} -I${CMAKE_SOURCE_DIR}/include -L${CMAKE_BINARY_DIR}/src
        -L${CMAKE_BINARY_DIR}/msg -lhaptix-comm -lhaptix-msgs
        -I${mex_ignition_transport_include} -L${mex_ignition_transport_lib}
        -lignition-transport0)

    if (WIN32)
      # Add Windows-specific libs
      set(MEX_FLAGS ${MEX_FLAGS} -lws2_32 -lIphlpapi)
    else()
      # Point the mex compiler to our custom config, which adjusts compiler
      # flags
      set(MEX_FLAGS ${MEX_FLAGS} -f ${CMAKE_CURRENT_SOURCE_DIR}/mexopts.sh)
    endif()
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/matlab)
    add_custom_command(
      OUTPUT ${MEX_OUTPUT_FILE}
      COMMAND ${MATLAB_MEX_PATH} ${MEX_FLAGS} ${CMAKE_CURRENT_SOURCE_DIR}/${mex_src}
      DEPENDS ${mex_srcs} ${PROJECT_NAME_LOWER} haptix-msgs
      COMMENT "Generating MATLAB mex file ${MEX_NAME}"
      VERBATIM)
  endforeach()

  install(FILES ${MEX_OUTPUT_FILE_SET}
                ${matlab_files}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/haptix-comm/matlab)

  if (WIN32)
    # To facilitate development, copy the zmq dll into the install tree in
    # the same directory as the .mex.  That way, no PATH-mangling is needed
    # to use the .mex in MATLAB.
    # Note that the SDK.zip is contructed through a different code path, with
    # various kinds of manual copying.
    # The filename construction is a little hacky.
    get_filename_component(ZMQ_DEBUG_DIR ${ZeroMQ_LIBRARY_DEBUG} DIRECTORY)
    get_filename_component(ZMQ_DEBUG_DLL ${ZeroMQ_LIBRARY_DEBUG} NAME_WE)
    set(ZMQ_DEBUG_DLL "${ZMQ_DEBUG_DIR}/../bin/${ZMQ_DEBUG_DLL}.dll")
    get_filename_component(ZMQ_RELEASE_DIR ${ZeroMQ_LIBRARY_RELEASE} DIRECTORY)
    get_filename_component(ZMQ_RELEASE_DLL ${ZeroMQ_LIBRARY_RELEASE} NAME_WE)
    set(ZMQ_RELEASE_DLL "${ZMQ_RELEASE_DIR}/../bin/${ZMQ_RELEASE_DLL}.dll")
    install(FILES ${ZMQ_RELEASE_DLL} ${ZMQ_DEBUG_DLL}
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/haptix-comm/matlab)
  endif()

  add_custom_target(matlab_generation ALL DEPENDS ${MEX_OUTPUT_FILE_SET})
endif()
