#===================================
#
# SimulationCore Project
#
#===================================
add_library(
    SimulationCore STATIC
    SimulationCore_pcp.h SimulationCore_pcp.cpp
    # Base Class
    ConstitutiveModel.h
    Mesh.h
    Object.h
    BC.h                          # Boundary conditions
    Model.h
    Step.h Step.cpp               # Simulation step
    Particle_Field.h
    TimeHistory.h TimeHistory.cpp # Time history output
    ResultFile.h ResultFile.cpp   # Result file
    # Common
    Mesh_R2D.hpp
    Object_Particle.hpp
    TimeHistory_ConsoleProgressBar.h TimeHistory_ConsoleProgressBar.cpp
    ResultFile_Text.h ResultFile_Text.cpp
    ResultFile_HDF5.h ResultFile_HDF5.cpp
    # Purely mechanics (Multiple Objects)
    Particle_2D_ME.h
    Object_Particle_2D_ME.h
    ParticleVar_R2D_ME.h
    Mesh_BG_R2D_ME.h Mesh_BG_R2D_ME.cpp
    NodeVar_2D_ME.h
    Model_R2D_ME_MPM.h Model_R2D_ME_MPM.cpp
    Step_R2D_ME_MPM.h  Step_R2D_ME_MPM.cpp
    TimeHistory_Particle_2D_ME_AllPcl.h TimeHistory_Particle_2D_ME_AllPcl.cpp
    # Coupled hydro-mechanics (Multiple Objects)
    # Particle_2D_CHM.h
    # Object_Particle_2D_CHM.h
    # ParticleVar_R2D_CHM.h
    # NodeVar_2D_CHM.h
    # Model_R2D_CHM_MPM.h Model_R2D_CHM_MPM.cpp
    # Step_R2D_CHM_MPM.h  Step_R2D_CHM_MPM.cpp
    # Purely mechanics (Single Object)
    Particle_R2D_ME_s.h
    Node_R2D_ME_s.h
    Element_R2D_ME_MPM_s.h
    Model_R2D_ME_MPM_s.h Model_R2D_ME_MPM_s.cpp
    Step_R2D_ME_MPM_s.h  Step_R2D_ME_MPM_s.cpp
    Step_R2D_ME_MPM_s_KinDamp.h Step_R2D_ME_MPM_s_KinDamp.cpp
    TimeHistory_Particle_R2D_ME_s.h TimeHistory_Particle_R2D_ME_s.cpp
    # Coupled Hydro-mechanics (Single Object)
    Particle_R2D_CHM_s.h
    Node_R2D_CHM_s.h
    Element_R2D_CHM_MPM_s.h
    Model_R2D_CHM_MPM_s.h Model_R2D_CHM_MPM_s.cpp 
    Step_R2D_CHM_MPM_s.h  Step_R2D_CHM_MPM_s.cpp
    Step_R2D_CHM_MPM_s_VisDamp.h Step_R2D_CHM_MPM_s_VisDamp.cpp
    Step_R2D_CHM_MPM_s_KinDamp.h Step_R2D_CHM_MPM_s_KinDamp.cpp
    TimeHistory_Particle_R2D_CHM_s.h TimeHistory_Particle_R2D_CHM_s.cpp
    # Purely mechanics (1D, B-spline, Single object)
    Model_1D_ME_MPM_BSpline_s.h Model_1D_ME_MPM_BSpline_s.cpp
    Step_1D_ME_MPM_BSpline_s.h Step_1D_ME_MPM_BSpline_s.cpp
    TimeHistory_Particle_1D_ME_MPM_BSpline_s_AllPcl.h TimeHistory_Particle_1D_ME_MPM_BSpline_s_AllPcl.cpp
    # Purely mechanics APIC
    Step_1D_ME_MPM_BSpline_APIC_s.h Step_1D_ME_MPM_BSpline_APIC_s.cpp
    # Purely mechanics (2D, b-spline, APIC)
    Model_R2D_ME_MPM_BSpline_s.h Model_R2D_ME_MPM_BSpline_s.cpp
    Step_R2D_ME_MPM_BSpline_APIC_s.h Step_R2D_ME_MPM_BSpline_APIC_s.cpp
    TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl.h TimeHistory_Particle_2D_ME_MPM_BSpline_s_AllPcl.cpp
    # Purely CHM (2D, b-spline, APIC)
    Model_R2D_CHM_MPM_BSpline_s.h Model_R2D_CHM_MPM_BSpline_s.cpp
    Step_R2D_CHM_MPM_BSpline_APIC_s.h Step_R2D_CHM_MPM_BSpline_APIC_s.cpp
    TimeHistory_Particle_2D_CHM_MPM_BSpline_s_AllPcl.h TimeHistory_Particle_2D_CHM_MPM_BSpline_s_AllPcl.cpp
    # Purely mechanics (complete set of quadriatic b-spline)
    Step_1D_ME_MPM_qBSpline_APIC_s.h Step_1D_ME_MPM_qBSpline_APIC_s.cpp
    # Purely mechanics GIMP
    Model_S2D_ME_MPM_s.h Model_S2D_ME_MPM_s.cpp
    Step_S2D_ME_MPM_s_GIMP.h Step_S2D_ME_MPM_s_GIMP.cpp
    TimeHistory_Particle_S2D_ME_AllPcl.h TimeHistory_Particle_S2D_ME_AllPcl.cpp
    # Coupled hydro-mechanics GIMP
    Model_S2D_CHM_MPM_s.h
    Step_S2D_CHM_MPM_s_GIMP.h Step_S2D_CHM_MPM_s_GIMP.cpp
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
