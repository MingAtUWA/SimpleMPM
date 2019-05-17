#===================================
#
# SimulationCore Project
#
#===================================
add_library(
    SimulationCore STATIC
    SimulationCore_pcp.h SimulationCore_pcp.cpp
    Model.h                       # Model.h
    BC.h                          # Boundary conditions
    Step.h Step.cpp               # Simulation step
    TimeHistory.h TimeHistory.cpp # Time history output
    ResultFile.h ResultFile.cpp   # Result file
    # Common
    TimeHistory_ConsoleProgressBar.h TimeHistory_ConsoleProgressBar.cpp
    ResultFile_Text.h ResultFile_Text.cpp
    ResultFile_HDF5.h ResultFile_HDF5.cpp
    # Purely mechanics
    Particle_2D_ME.h Node_R2D_ME.h Element_R2D_ME_MPM.h
    Model_R2D_ME_MPM_s.h Model_R2D_ME_MPM_s.cpp
    Step_R2D_ME_MPM_s.h Step_R2D_ME_MPM_s.cpp
    Step_R2D_ME_MPM_s_KinDamp.h Step_R2D_ME_MPM_s_KinDamp.cpp
    TimeHistory_Particle_2D_ME.h TimeHistory_Particle_2D_ME.cpp
    # Coupled Hydro-mechanics
    Particle_2D_CHM.h Node_R2D_CHM.h Element_R2D_CHM_MPM.h
    Model_R2D_CHM_MPM_s.h Model_R2D_CHM_MPM_s.cpp 
    Step_R2D_CHM_MPM_s.h Step_R2D_CHM_MPM_s.cpp
    Step_R2D_CHM_MPM_s_damp.h Step_R2D_CHM_MPM_s_damp.cpp
    Step_R2D_CHM_MPM_s_KinDamp.h Step_R2D_CHM_MPM_s_KinDamp.cpp
    TimeHistory_Particle_2D_CHM.h TimeHistory_Particle_2D_CHM.cpp
    )

target_include_directories(
    SimulationCore PRIVATE
    ${CMAKE_SOURCE_DIR}/Utilities/
    ${HDF5_INCLUDE_DIR}
    )

link_directories(${HDF5_LIBRARIES_DIR})
target_link_libraries(
    SimulationCore PRIVATE
    Utilities
    ${HDF5_LIBRARIES}
    )

if (WIN32)
    # supress warning for certain functions in visual studio
    target_compile_definitions(SimulationCore PRIVATE _CRT_SECURE_NO_WARNINGS)
    # use precompiled header
    target_compile_options(SimulationCore PRIVATE -YcSimulationCore_pcp.h)
endif()
