# Libigl linking
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_USE_STATIC_LIBRARY "Use libIGL as static library" ON)
option(LIBGL_COMMON ON)
# find_package(LIBIGL REQUIRED QUIET)
include(libigl)


# GMSH linking
find_library(GMSH_LIB gmsh)
if(NOT GMSH_LIB)
  message(FATAL_ERROR "Could not find libgmsh")
endif()

find_path(GMSH_INC gmsh.h)
if(NOT GMSH_INC)
  message(FATAL_ERROR "Could not find gmsh.h")
endif()

# Dynamics math eigen linking
find_package(dynamics_math_eigen CONFIG REQUIRED)

# Include directories
include_directories(
  /usr/local/include/eigen3
  ./include/
)

# Libraries
set(ALL_LIBS
  igl::core
  dme::dynamics_math_eigen ${GMSH_LIB})

add_library(needle_tissue_simulator STATIC
  ./src/needle_tissue_simulator.cpp
  ./src/tissue_model.cpp
  ./src/needle_model.cpp
  ./src/mesh3D.cpp
  ./src/tissue_mesh.cpp
  ./src/box_structured_mesh.cpp
  ./src/tetrahedral_meshing.cpp
  ./src/force_interface.cpp
  ./src/cosserat_xpbd.cpp
  ./src/xpbd_constraints.cpp
  ./src/tetrahedron_collider.cpp
  ./src/collisions_interface.cpp
  ./src/needle_tissue_interface.cpp
  ./src/friction_model.cpp
  ./src/utils.cpp
)

target_include_directories(needle_tissue_simulator PRIVATE ${GMSH_INC})

target_link_libraries(needle_tissue_simulator ${ALL_LIBS})