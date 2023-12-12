cmake_minimum_required( VERSION 3.10 )

project(Sawing_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Optimization parameters
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Os")

# OpenMP linking
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Dynamics math eigen linking
find_package(dynamics_math_eigen CONFIG REQUIRED)

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

# Autodiff
find_package(autodiff REQUIRED)

# Add third party directory
add_subdirectory(libs)

# Include directories
include_directories(
    /usr/local/include/eigen3
    ./include  
)

set(ALL_LIBS
    third_party 
    igl::core
    autodiff::autodiff
    dme::dynamics_math_eigen 
    ${GMSH_LIB}
  )

set(SOURCES
    ./src/animation_interface.cpp
    ./src/application_interface.cpp
    ./src/user_interface.cpp
    ./src/joystick_interface.cpp
    ./src/world_interface.cpp
    # ./src/video_recorder.cpp
  )
  
# target
add_executable(main main.cpp ${SOURCES})

target_include_directories(main PRIVATE ${GMSH_INC})

target_link_libraries(main ${ALL_LIBS})