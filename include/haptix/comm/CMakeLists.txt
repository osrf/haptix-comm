include (${project_cmake_dir}/Utils.cmake)

set (headers
  haptix.h
  haptix_sim.h
)

#################################################
### Do not change the following
set (ign_headers "" CACHE INTERNAL "Ignition headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Ignition headers" "#include <ignition/${IGN_PROJECT_NAME}/${hdr}>\n")
endforeach()

ign_install_includes(${IGN_PROJECT_NAME} ${headers})
