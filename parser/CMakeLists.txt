# #############################################################################
# LIBRARY NAME
# #############################################################################
set(LIBRARY_NAME cloud_parser)

find_package(PCL 1.8 REQUIRED QUIET)

if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
    add_definitions(${PCL_DEFINITIONS})
else()
    message(FATAL_ERROR "   ERROR: PCL minimum required version 1.8. Not found")
endif()

# #############################################################################
# COMPILE LIBRARY
# #############################################################################
add_library(${LIBRARY_NAME} SHARED
    parser.cpp
)

# #############################################################################
# LIBRARY PROPERTIES
# #############################################################################
set_target_properties(${LIBRARY_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/parser/include/modern/parser.hpp
)

# #############################################################################
# LIBRARY HEADERS
# #############################################################################
target_include_directories(${LIBRARY_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/parser/include
    ${PCL_INCLUDE_DIRS}

    # ${VTK_INCLUDE_DIRS}
)

# #############################################################################
# LIBRARY DEPENDENCIES
# #############################################################################
target_link_libraries(${LIBRARY_NAME} PUBLIC ${PCL_LIBRARIES})

# #############################################################################
# COMPILATION FLAGS
# #############################################################################
target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)
target_compile_options(${LIBRARY_NAME} PRIVATE -Wno-cpp)

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${LIBRARY_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)