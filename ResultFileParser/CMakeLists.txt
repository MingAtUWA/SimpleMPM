#===================================
#
# ResultFileParser Project
#
#===================================
add_library(
    ResultFileParser STATIC 
    FileCharArray.h
    ParserException.h
    TextResultFileInfo.h TextResultFileInfo.cpp    
    FieldDataParser.hpp
    TextResultFileParser.hpp
    )
           
target_include_directories(
    ResultFileParser PRIVATE
    ${CMAKE_SOURCE_DIR}/Utilities/
    )
    
if (WIN32)
    # supress warning for certain functions in visual studio
    target_compile_definitions(ResultFileParser PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
