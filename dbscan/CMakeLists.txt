# #############################################################################
# LIBRARY NAME
# #############################################################################
cmake_minimum_required(VERSION 3.12.4)
project(dbscan
    VERSION 1.0.3
    DESCRIPTION "dbscan library"
    LANGUAGES CXX
)

set(Eigen3_DIR /usr/share/cmake/Modules)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS octree kdtree)

if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
else()
    message(FATAL_ERROR "   ERROR: PCL minimum required version 1.8. Not found")
endif()

# #############################################################################
# COMPILE LIBRARY
# #############################################################################
add_library(${PROJECT_NAME} SHARED
    src/cluster.cpp
    src/dbScan.cpp
    src/OctreeGenerator.cpp
)

# #############################################################################
# LIBRARY PROPERTIES
# #############################################################################
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER ${CMAKE_CURRENT_LIST_DIR}/include/dbscan/dbScan.h
)

# #############################################################################
# LIBRARY HEADERS
# #############################################################################
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

# #############################################################################
# LIBRARY DEPENDENCIES
# #############################################################################
# target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_KDTREE_LIBRARY} ${PCL_OCTREE_LIBRARY} Eigen3::Eigen)

# #############################################################################
# COMPILATION FLAGS: MMX, SSE(1, 2, 3, 3S, 4.1, 4.2), CLMUL, RdRand, VT-x, x86-64
# #############################################################################
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp
    -mmmx
    -msse
    -msse2
    -msse3
    -mssse3
    -msse4.2
    -msse4.1
    -mno-sse4a
    -mno-avx
    -mno-avx2
    -mno-fma
    -mno-fma4
    -mno-f16c
    -mno-xop
    -mno-bmi
    -mno-bmi2
    -mrdrnd
    -mno-3dnow
    -mlzcnt
    -mfsgsbase
    -mpclmul
)

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)