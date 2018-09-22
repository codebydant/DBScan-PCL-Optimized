# DBScan-PCL-Optimized

This project is taken from: Navarro-Hinojosa, Octavio, y Moisés Alencastre-Miranda. "DBSCAN modificado con Octrees para agrupar nubes de puntos en tiempo real." 
Research in Computing Science, Vol. 114: Advances in Image Processing and Computer Vision, pp. 173–186, 2016.

News:
* Added CMakeList.txt for cmake compile with PCL 1.8.1
* Added arguments param option
* Added pcl visualizer
* Deleted Glut visualizer
* Added cluster saving method
* Added cluster coloring method
* Replaced the input file from csv to txt

## Input file structure

* PCD
* PLY
* TXT: x y z
* XYZ

-------------------
## Compile
* Create a "build" folder

in the main folder:

    - cd build  
    - cmake ../
    - make
       
        	 
### Test

    ./dbscan <txt file> <eps> <min Pts> <max Pts>
    eps = 40
    min Pts = 10
    max Pts = 1000
    txt file = path to data: worldCloud15.txt



