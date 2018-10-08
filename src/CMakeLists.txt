##############################################################################
# CMAKE CONFIGURATION
##############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# set project name
project(dbscan VERSION 1.0.0)

# set build type = Debug mode
set(CMAKE_BUILD_TYPE Release)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# set the include directive in the same project folder
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set corresponding package directories
set(PCL_DIR /opt/pcl-1.8.1/build)


# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# set turn off the output rule messages of cmake
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

##############################################################################
# PACKAGES
##############################################################################
message("Finding packages...")

message("***********************")
message("PCL PACKAGE")
message("***********************")

find_package(PCL 1.8.1 REQUIRED PATHS ${PCL_DIR})
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: PCL not found")
endif()
find_package(OpenGL)


##############################################################################
# HEADERS
##############################################################################
include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OPENGL_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
#add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${PCL_LIBRARY_DIRS})

##############################################################################
# SOURCE CODE
##############################################################################

set(MAIN_SOURCE "dbScanMain.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE} "dbScan.cpp" "OctreeGenerator.cpp" "cluster.cpp" "dbScan.h" "OctreeGenerator.h" "cluster.h" "mouseUtils.h" "HTRBasicDataStructures.h")

##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} )

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")
