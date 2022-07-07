# #############################################################################
# LIBRARY NAME
# #############################################################################
cmake_minimum_required(VERSION 3.12.4)
project(dbscan
    VERSION 1.0.3
    DESCRIPTION "dbscan library"
    LANGUAGES CXX
)

find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io common octree visualization kdtree)

if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")

    add_definitions(${PCL_DEFINITIONS})

    # add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)

    # # add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
    # add_definitions(-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)
    message(STATUS "    PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
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

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "    clang compiler enabled")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "    gnu compiler enabled")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    message(STATUS "    intel compiler enabled")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message(STATUS "    msvc compiler enabled")
endif()

# #############################################################################
# LIBRARY DEPENDENCIES
# #############################################################################
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
message(STATUS "    COMPILE_OPTIONS: ${COMPILE_OPTIONS}")

# #############################################################################
# COMPILATION FLAGS
# #############################################################################
# add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)

# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp -mmmx -mno-3dnow -msse -msse2 -msse3 -mssse3 -mno-sse4a -mcx16 -msahf -mmovbe -mno-aes -mno-sha -mpclmul -mpopcnt -mno-lwp -mno-fma -mno-fma4 -mno-xop -mno-bmi -mno-sgx -mno-bmi2 -mno-pconfig -mno-wbnoinvd -mno-tbm -mno-avx -mno-avx2 -msse4.2 -msse4.1 -mlzcnt -mno-rtm -mrdrnd -mno-f16c -mfsgsbase -mno-rdseed -mno-prfchw -mno-adx -mfxsr -mno-xsave -mno-xsaveopt -mno-avx512f -mno-avx512er -mno-avx512cd -mno-avx512pf -mno-prefetchwt1 -mno-clflushopt -mno-xsavec -mno-xsaves -mno-avx512dq -mno-avx512bw -mno-avx512vl -mno-avx512ifma -mno-avx512vbmi -mno-clwb -mno-mwaitx -mno-clzero -mno-pku -mno-rdpid -mno-gfni -mno-shstk -mno-avx512vbmi2 -mno-avx512vnni -mno-vaes -mno-vpclmulqdq -mno-avx512bitalg -mno-avx512vpopcntdq -mno-movdiri -mno-movdir64b -mno-waitpkg -mno-cldemote -mno-ptwrite
# )
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp -mmmx -msse -mno-avx)

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)