#===================================
#
# Utilities Project
#
#===================================
add_library(
    Utilities STATIC
    Utilities_pcp.h Utilities_pcp.cpp
    common_utilities.h common_utilities.cpp
    ItemArray.hpp
    ItemBuffer.hpp
    ItemStack.hpp
    ItemList.hpp
    LinkedMemoryPool.h
    ContinuousFixedSizeMemory.h
    ContinuousFlexibleSizeMemory.h
    LinkedFixedSizeMemory.h LinkedFixedSizeMemory.cpp
    LinkedFlexibleSizeMemory.h
    MemoryBuffer.h
    PreAllocStringBuffer.hpp
    StringBuffer.h
    StackLikeBuffer.hpp
    # Rigid body
    Geometry.h Geometry.cpp
    HashTable.h HashTableEdge.h
    TriangleMesh.h TriangleMesh.cpp TriangleMesh_BgGrid.cpp
    RigidBody.h RigidBody.cpp
    DrawTriangleMesh.h DrawTriangleMesh.cpp
    # OpenGL
    OpenGL_headers.h
    glad.c
    # shader
    ShaderProgram.h ShaderProgram.cpp
    )

target_include_directories(
    Utilities PRIVATE
    ${GLAD_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${GLM_INCLUDE_DIR}
    )

link_directories(${GLFW_LIBRARIES_DIR})
target_link_libraries(
    Utilities PRIVATE
    ${GLFW_LIBRARIES}
    )

if (WIN32)
    # supress warning for certain functions in visual studio
    target_compile_definitions(Utilities PRIVATE _CRT_SECURE_NO_WARNINGS)
    # use precompiled header
    target_compile_options(Utilities PRIVATE -YcUtilities_pcp.h)
    # exclude glad.c
    set_source_files_properties(glad.c
        PROPERTIES
        COMPILE_FLAGS "/Y-"
        )
endif()
