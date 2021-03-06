find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/haptix.in
                 ${CMAKE_BINARY_DIR}/haptix.dox @ONLY)

  add_custom_target(doc

    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/haptix.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/search.js
            ${CMAKE_BINARY_DIR}/doxygen/html/search

    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/mpl_joints.svg
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/mpl_motors.svg
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/mpl_contact_sensors.svg
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/mpl_imu_sensors.svg
            ${CMAKE_BINARY_DIR}/doxygen/html

    # Refer the Readme file for instructions on updating these files.
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/hx_update.html
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/hx_robot_info.html
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/hx_read_sensors.html
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/hx_connect.html
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/hx_close.html
            ${CMAKE_BINARY_DIR}/doxygen/html

    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()
