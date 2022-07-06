# #############################################################################
# LIBRARY NAME
# #############################################################################
cmake_minimum_required(VERSION 3.12.4)
project(dbscan
    VERSION 1.0.3
    DESCRIPTION "dbscan library"
    LANGUAGES CXX
)
set(ENABLE_AVX OFF)
set(ENABLE_AVX2 OFF)
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
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

# #############################################################################
# COMPILATION FLAGS
# #############################################################################
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-cpp -imultiarch x86_64-linux-gnu - -march=haswell -mmmx -mno-3dnow -msse -msse2 -msse3 -mssse3 -mno-sse4a -mcx16 -msahf -mmovbe -mno-aes -mno-sha -mpclmul -mpopcnt -mabm -mno-lwp -mno-fma -mno-fma4 -mno-xop -mno-bmi -mno-sgx -mno-bmi2 -mno-pconfig -mno-wbnoinvd -mno-tbm -mno-avx -mno-avx2 -msse4.2 -msse4.1 -mlzcnt -mno-rtm -mno-hle -mrdrnd -mno-f16c -mfsgsbase -mno-rdseed -mno-prfchw -mno-adx -mfxsr -mno-xsave -mno-xsaveopt -mno-avx512f -mno-avx512er -mno-avx512cd -mno-avx512pf -mno-prefetchwt1 -mno-clflushopt -mno-xsavec -mno-xsaves -mno-avx512dq -mno-avx512bw -mno-avx512vl -mno-avx512ifma -mno-avx512vbmi -mno-avx5124fmaps -mno-avx5124vnniw -mno-clwb -mno-mwaitx -mno-clzero -mno-pku -mno-rdpid -mno-gfni -mno-shstk -mno-avx512vbmi2 -mno-avx512vnni -mno-vaes -mno-vpclmulqdq -mno-avx512bitalg -mno-avx512vpopcntdq -mno-movdiri -mno-movdir64b -mno-waitpkg -mno-cldemote -mno-ptwrite --param l1-cache-size=32 --param l1-cache-line-size=64 --param l2-cache-size=3072 -mtune=haswell -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection
)

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)