cmake_minimum_required(VERSION 3.9)

project(cloud_parser VERSION 1.0.3)

find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io common octree visualization kdtree)

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
add_library(${PROJECT_NAME} SHARED
    parser.cpp
)

# #############################################################################
# LIBRARY PROPERTIES
# #############################################################################
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/parser/include/modern/parser.hpp
)

# #############################################################################
# LIBRARY HEADERS
# #############################################################################
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}

    # ${VTK_INCLUDE_DIRS}
)

# #############################################################################
# LIBRARY DEPENDENCIES
# #############################################################################
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

# #############################################################################
# COMPILATION FLAGS
# #############################################################################
# target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)
# target_compile_options(${LIBRARY_NAME} PUBLIC -Wno-cpp)

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)