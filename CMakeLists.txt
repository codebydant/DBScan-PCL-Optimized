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

# # set the include directive in the same project folder
# set(CMAKE_INCLUDE_CURRENT_DIR ON)

# # set corresponding package directories
# set(PCL_DIR /opt/pcl-1.9.0/build)

# # Include dependencies from pcl 1.8.0 in project directory
# set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS "-mno-avx")
SET(CMAKE_C_COMPILER "clang")
SET(CMAKE_CXX_COMPILER "clang++")

# set turn off the output rule messages of cmake
# #set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
# find_package(PkgConfig)
# pkg_check_modules(PC_FLANN flann)
# set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

# find_path(FLANN_INCLUDE_DIR flann/flann.hpp
# HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS})

# find_library(FLANN_LIBRARY flann
# HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})

# set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
# set(FLANN_LIBRARIES ${FLANN_LIBRARY})

# #############################################################################
# PACKAGES
# #############################################################################
# message("***********************")
# message("PCL PACKAGE")
# message("***********************")

# #find_package(PCL 1.8 PATHS ${PCL_DIR} QUIET COMPONENTS common io visualization CONFIG) ##REQUIRED COMPONENTS common io visualization)
# find_package(PCL 1.8 REQUIRED QUIET)

# if(PCL_FOUND)
# message(STATUS "${BoldGreen}PCL status:")
# message(STATUS "    version: ${PCL_VERSION}")
# message(STATUS "    directory: ${PCL_DIR}")
# else()
# message(FATAL_ERROR "${BoldRed}    ERROR: PCL minimum required version 1.8. Not found")
# endif()

# message("${BoldYellow}***********************")
# message("${BoldYellow}VTK PACKAGE")
# message("${BoldYellow}***********************")
find_package(VTK 8.1 REQUIRED QUIET)

if(VTK_FOUND)
    message(STATUS "${BoldGreen}VTK status:")
    message(STATUS "    version: ${VTK_VERSION}")
    message(STATUS "    directory: ${VTK_DIR}")
else()
    message(FATAL_ERROR "${BoldRed} ERROR: VTK minimum required version 8.1. Not found")
endif()

# Use the compile definitions defined in PCL
# add_definitions(${PCL_DEFINITIONS})

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
# target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS} include)
# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} cloud_parser dbscan argparse)
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
