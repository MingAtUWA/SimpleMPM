#===================================
#
# TestsWithGL Project
#
#===================================
add_executable(
    TestsWithGL TestsWithGL_main.cpp
    TestsWithGL_pcp.h TestsWithGL_pcp.cpp
    test_sim_core.h
    test_post_processor.h
    # Test functions
    # rigid body testing
    display_triangle_mesh.cpp
    test_triangle_mesh_circle.cpp
    test_triangle_mesh_square.cpp
    test_rigid_body_square.cpp
    # mpm - rigid body contact
    test_mpm_rigidbody_circle.cpp
    test_mpm_rigidbody_square.cpp
    )

target_include_directories(
    TestsWithGL PRIVATE
    ${CMAKE_SOURCE_DIR}/SimulationCore/
    ${CMAKE_SOURCE_DIR}/PostProcessor/
    ${CMAKE_SOURCE_DIR}/ResultFileParser/
    ${CMAKE_SOURCE_DIR}/Utilities/
    ${GLAD_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${GLM_INCLUDE_DIR}
    )

link_directories(${GLFW_LIBRARIES_DIR})
    
target_link_libraries(
    TestsWithGL PRIVATE
    SimulationCore
    PostProcessor
    ResultFileParser
    Utilities
    ${GLFW_LIBRARIES}
    )

if (WIN32)
    # supress warning for certain functions in visual studio
    target_compile_definitions(TestsWithGL PRIVATE _CRT_SECURE_NO_WARNINGS)
    # use precompiled header
    target_compile_options(TestsWithGL PRIVATE -YcTestsWithGL_pcp.h)
endif()
