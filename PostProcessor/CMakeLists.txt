# ===================================
#
# PostProcessor project
#
# ===================================

add_library(
    PostProcessor STATIC 
    PostProcessor_pcp.h PostProcessor_pcp.cpp
    Frame_Particle2D.h Frame_Particle2D.cpp
    Animation.h Animation.cpp
    Animation_Test.h Animation_Test.cpp
    Animation_Particle2D.h Animation_Particle2D.cpp
    Animation_Particle2D_RigidBody.h Animation_Particle2D_RigidBody.cpp
    screenshot.h screenshot.cpp
    gif.h
    )

target_include_directories(
    PostProcessor PRIVATE
    ${CMAKE_SOURCE_DIR}/ResultFileParser/
    ${CMAKE_SOURCE_DIR}/Utilities/
    )

target_link_libraries(PostProcessor ${VTK_LIBRARIES} Utilities)

if (WIN32)
    target_compile_definitions(PostProcessor PRIVATE _CRT_SECURE_NO_WARNINGS)
    # use precompiled header "pcp.h"
    target_compile_options(PostProcessor PRIVATE -YcPostProcessor_pcp.h)
endif()
