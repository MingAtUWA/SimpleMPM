cmake_minimum_required(VERSION 3.0)

#===================================
#
# SimpleMPM Project
#
#===================================
project(SimpleMPM)

set(VENDORS_DIR "${CMAKE_SOURCE_DIR}/Vendors")

# HDF5 Package
# use find_package in the future?
set(HDF5_INCLUDE_DIR "${VENDORS_DIR}/hdf5/include")
set(HDF5_LIBRARIES_DIR "${VENDORS_DIR}/hdf5/libs")
find_library(HDF5_LIBRARIES
    NAMES libhdf5_D libhdf5_cpp_D libhdf5_hl_D libhdf5_hl_cpp_D libhdf5_tools_D
    PATHS ${HDF5_LIBRARIES_DIR} 
    )

# VTK Package
set(VTK_DIR "C:/Users/22671944/Vendors/VTK/lib/cmake/vtk-8.2")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# SimulationCore
add_subdirectory(SimulationCore/)

# PostProcessor
add_subdirectory(PostProcessor/)

# ResultFileParser
add_subdirectory(ResultFileParser/)

# Utilities
add_subdirectory(Utilities/)

# Test
add_subdirectory(Tests/)
