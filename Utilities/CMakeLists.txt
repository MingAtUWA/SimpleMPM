#===================================
#
# Utilities Project
#
#===================================
add_library(
    Utilities STATIC
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
    )

if (WIN32)
    # supress warning for certain functions in visual studio
    target_compile_definitions(Utilities PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
