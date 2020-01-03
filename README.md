# DBScan-PCL-Optimized

This project is taken from: **Navarro-Hinojosa, Octavio, y Moisés Alencastre-Miranda. "DBSCAN modificado con Octrees para agrupar nubes de puntos en tiempo real." Research in Computing Science, Vol. 114: Advances in Image Processing and Computer Vision, pp. 173–186, 2016.** Github: https://github.com/Hagen23/DBScan_Octrees

It was modified with:

* It was added a CMakeList.txt for cmake compilation with PCL 1.9.0 (support 1.9.1)
* It was added an argument param option
* It was added a pcl visualizer
* It was deleted the Glut visualizer
* It was added a cluster saving method
* It was added a cluster coloring method
* It was replaced the input file from CSV to PCD 
* It was added a cluster coloring method for original color of the point cloud

## Input file structure support

* .pcd 
* .ply
* .txt
* .xyz

## Output file structure (default = .pcd)
* cloud_cluster_#.txt: 

        x y z r g b

## Example
<img src="./example/scan1.png" align="center" height="400" width="720"><br>
<img src="./example/example2.png" align="center" height="400" width="720"><br>

-------------------
## Compilation
* Set "YOUR OWN" PCL Build DIR in CMakeList.txt e.g: **/opt/pcl-1.8.1/build** and save it.
* Create a "build" folder

in the main folder:

    - cd build/  
    - cmake ../
    - make
       
        	 
### Test

    cd build/bin
    ./dbscan <input file>       <-- This will run a fast testing with default parameters
    ./dbscan <input file> <octree resolution> <eps> <min Aux Pts> <min Pts> <output dir> <output extension (optional)>
    
    input file = path to input point_cloud (.pcd .txt .ply .xyz)
    octree resolution = 124
    eps = 40
    min Pts = 4
    max Pts = 5
    output dir = path to output clusters
    output extension (optional) = pcd, ply, txt or xyz
    
    Example:
    ./dbscan /home/xXx/Downloads/point_cloud.pcd 124 40 5 5 /home/xXx/Downloads/clusters ply     
    
    ¡You can modify the parameters to obtain better results!
    I recommend modifying only the eps value, with 40 - 60 you can get better clusters.
    
## Note

If you do not want to see the output clusters on PCL Viewer, set: 

    bool showClusters = False; //Main function
    
## Troubleshoot PCL-1.9.1
if compiling the project with PCL-1.9.1 this occurs:

        -- Build files have been written to: /home/t00215031/Downloads/DBScan-PCL-Optimized-master/build
        [ 20%] Building CXX object CMakeFiles/dbscan.dir/main.cpp.o
        In file included from /opt/pcl-1.9.1/common/include/pcl/pcl_macros.h:75:0,
                         from /opt/pcl-1.9.1/octree/include/pcl/octree/octree_nodes.h:47,
                         .
                         .
                         .

        /opt/pcl-1.9.1/build/include/pcl/pcl_config.h:7:4: error: #error PCL requires C++14 or above
           #error PCL requires C++14 or above
            ^
        CMakeFiles/dbscan.dir/build.make:62: recipe for target 'CMakeFiles/dbscan.dir/main.cpp.o' failed
        make[2]: *** [CMakeFiles/dbscan.dir/main.cpp.o] Error 1
        CMakeFiles/Makefile2:67: recipe for target 'CMakeFiles/dbscan.dir/all' failed
        make[1]: *** [CMakeFiles/dbscan.dir/all] Error 2
        Makefile:83: recipe for target 'all' failed
        make: *** [all] Error 2
        
        ## Solution
        1. Update gcc and g++ to version 6:
           $    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
           $    sudo apt-get update -y
           $    sudo apt-get install -y gcc-6 g++-6 clang-3.8
           $    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 70 --slave /usr/bin/g++ g++ /usr/bin/g++-6
           
           --> check gcc and g++ version:
           $    gcc --version
           $    g++ --version
           
                g++ (Ubuntu 6.5.0-2ubuntu1~16.04) 6.5.0 20181026
                Copyright (C) 2017 Free Software Foundation, Inc.
                This is free software; see the source for copying conditions.  There is NO
                warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
                
         2. Compile again 
            $    cmake../ && make
         
if: 

        ../../../bin/librtabmap_core.so.0.11.11: undefined reference to `pcl::search::Search<pcl::PointXYZRGBNormal>::getName[abi:cxx11]() const'
        ../../../bin/librtabmap_core.so.0.11.11: undefined reference to `pcl::search::Search<pcl::PointXYZRGB>::getName[abi:cxx11]() const'
        ../../../bin/librtabmap_core.so.0.11.11: undefined reference to `pcl::search::Search<pcl::PointXYZ>::getName[abi:cxx11]() const'
        collect2: error: ld returned 1 exit status
        
        Solution:

                #include <pcl/search/impl/search.hpp>

                #ifndef PCL_NO_PRECOMPILE
                #include <pcl/impl/instantiate.hpp>
                #include <pcl/point_types.h>
                PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
                #endif // PCL_NO_PRECOMPILE

           

                
        
        
      
     

    



