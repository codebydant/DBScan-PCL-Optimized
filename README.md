# DBScan-PCL-Optimized

This project is taken from: **Navarro-Hinojosa, Octavio, y Moisés Alencastre-Miranda. "DBSCAN modificado con Octrees para agrupar nubes de puntos en tiempo real." Research in Computing Science, Vol. 114: Advances in Image Processing and Computer Vision, pp. 173–186, 2016.** Github: https://github.com/Hagen23/DBScan_Octrees

It was modified with:

* It was added a CMakeList.txt for cmake compilation with PCL 1.8.1 (support 1.9.1)
* It was added an argument param options
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

    - cd build  
    - cmake ../
    - make
       
        	 
### Test

    cd /build/bin
    ./dbscan <pcd file> <octree resolution> <eps> <min Aux Pts> <min Pts> <output dir> <output extension (optional)>
    
    pcd file = path to point_cloud.pcd
    octree resolution = 124
    eps = 40
    min Pts = 4
    max Pts = 5
    output dir = path to save
    output extension (optional) = pcd (default) --> you can set ply, txt or xyz
    
    Example:
    ./dbscan /home/xXx/Downloads/point_cloud.pcd 124 40 5 5 /home/xXx/Downloads/clusters     
    
    ¡You can modify the parameters to obtain better results!
    I recommend modifying only the eps value, with 40 - 60 you can get better clusters.
    



