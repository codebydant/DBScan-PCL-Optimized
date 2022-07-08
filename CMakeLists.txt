# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# set project name
project(app VERSION 1.0.3)

# set build type = Debug mode
set(CMAKE_BUILD_TYPE Debug)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/color.cmake")

message("\n" "${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} ")
message("${BoldCyan}=========================================")

# set cmake for use std c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# #############################################################################
# PACKAGES
# #############################################################################
message("${BoldYellow}***********************")
message("${BoldYellow}VTK PACKAGE")
message("${BoldYellow}***********************")
find_package(VTK 8.1 REQUIRED QUIET)

if(VTK_FOUND)
    message(STATUS "${BoldGreen}VTK status:")
    message(STATUS "    version: ${VTK_VERSION}")
    message(STATUS "    directory: ${VTK_DIR}")
else()
    message(FATAL_ERROR "${BoldRed} ERROR: VTK minimum required version 8.1. Not found")
endif()

# #############################################################################
# ADD LIBRARIES FOLDER
# #############################################################################
add_subdirectory(dbscanlib)
add_subdirectory(parser)
add_subdirectory(external/argparse)

# #############################################################################
# SOURCE CODE
# #############################################################################
set(MAIN_SOURCE "main.cpp")

# #############################################################################
# EXECUTABLES
# #############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

# #############################################################################
# TARGET LIBRARIES
# #############################################################################
target_include_directories(${PROJECT_NAME} PRIVATE ${VTK_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} argparse dbscan cloudparse)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp)

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
