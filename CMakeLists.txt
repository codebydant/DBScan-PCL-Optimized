# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/color.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

# set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "Final installation location.")
set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set project name
project(app VERSION 1.0.6)

message("\n" "${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} ")
message("${BoldCyan}=========================================")

# #############################################################################
# PACKAGES/DEPENDENCIES
# #############################################################################
# vtkRenderingContext2D vtkRenderingOpenGL2 vtkRenderingCore
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS visualization)
find_package(VTK 8.1 REQUIRED QUIET COMPONENTS vtkRenderingContextOpenGL2)
fetch_project(
    NAME cloudparse
    URL https://github.com/danielTobon43/cloudparse/archive/v0.2.1.tar.gz
)
fetch_project(
    NAME argparse
    URL https://github.com/p-ranav/argparse/archive/v2.6.tar.gz
)

# #############################################################################
# ADD LIBRARIES FOLDER
# #############################################################################
add_subdirectory(dbscan)

# #############################################################################
# SOURCE CODE
# #############################################################################
set(MAIN_SOURCE "main.cpp")

# #############################################################################
# EXECUTABLES
# #############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

# #############################################################################
# TARGET HEADERS
# #############################################################################
target_include_directories(${PROJECT_NAME} PRIVATE ${VTK_INCLUDE_DIRS} include)

# #############################################################################
# TARGET LIBRARIES
# #############################################################################
target_link_libraries(${PROJECT_NAME} argparse dbscan cloudparse ${VTK_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

# #############################################################################
# TARGET COMPILE OPTIONS
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

# vtk_module_autoinit(TARGETS ${PROJECT_NAME} MODULES VTK::RenderingContextOpenGL2)
message("${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("${BoldCyan}=========================================")

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
