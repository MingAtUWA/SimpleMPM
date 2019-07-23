#===================================
#
# Tests Project
#
#===================================
add_executable(
    Tests Test_main.cpp
    Test_pcp.h Test_pcp.cpp
    test_sim_core.h
    test_post_processor.h
    # simulation core
    display_model.cpp
    test_mesh.cpp
    test_cal_shape_func.cpp
    test_me_mpm1.cpp
    test_me_mpm2.cpp
    test_me_mpm3.cpp
    test_me_mpm4.cpp
    test_me_mpm5.cpp
    test_me_kindamp_mpm1.cpp
    test_me_kindamp_mpm2.cpp
    test_me_1D_bspline_mpm1.cpp
    test_me_1D_bspline_mpm2.cpp
    test_me_2D_bspline_mpm1.cpp
    test_me_2D_bspline_mpm2.cpp
    test_me_2D_bspline_mpm3.cpp
    test_me_mpm_gimp1.cpp
    test_me_mpm_gimp2.cpp
    test_chm_mpm1.cpp
    test_chm_mpm2.cpp
    test_chm_visdamp_mpm1.cpp
    test_chm_visdamp_mpm2.cpp
    test_chm_visdamp_mpm3.cpp
    test_chm_visdamp_mpm4.cpp
    test_chm_kindamp_mpm2.cpp
    test_chm_2D_bspline_mpm1.cpp
    test_chm_2D_bspline_mpm2.cpp
    test_Model_S2D.cpp
    # multiple object
    test_multi_object_me1.cpp
    # write result file
    test_hdf5_resultfile.cpp
    test_text_resultfile.cpp
    # window system
    test_frame_pcl2d.cpp
    test_animation.cpp
    test_animation_particle2d.cpp
    # read result file
    test_result_file_parser.cpp
    test_file_char_array.cpp
    # utilities
    test_item_array.cpp
    test_StackLikeBuffer.cpp
    )

target_include_directories(
    Tests PRIVATE
    ${CMAKE_SOURCE_DIR}/SimulationCore/
    ${CMAKE_SOURCE_DIR}/PostProcessor/
    ${CMAKE_SOURCE_DIR}/ResultFileParser/
    ${CMAKE_SOURCE_DIR}/Utilities/
    ${HDF5_INCLUDE_DIR}
    )

link_directories(${HDF5_LIBRARIES_DIR})
target_link_libraries(
    Tests PRIVATE
    SimulationCore
    PostProcessor
    ResultFileParser
    Utilities
    ${VTK_LIBRARIES}
    ${HDF5_LIBRARIES}
    )

if (WIN32)
    # supress warning for certain functions in visual studio
    target_compile_definitions(Tests PRIVATE _CRT_SECURE_NO_WARNINGS)
    # use precompiled header
    target_compile_options(Tests PRIVATE -YcTest_pcp.h)
endif()
