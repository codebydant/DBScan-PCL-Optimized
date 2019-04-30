#define _CRT_SECURE_NO_WARNINGS

#include "src/OctreeGenerator.h"
#include "src/dbScan.h"
#include "src/HTRBasicDataStructures.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/thread/thread.hpp>


// Colors to display the generated clusters
float colors[] = {  255,0,0,  //red 		1
                    0,255,0,  //green		2
                    0,0,255,  //blue		3
                    255,255,0,//yellow		4
                    0,255,255,//light blue	5
                    255,0,255,//magenta     6               
                    255,255,255,//white		7
                    255,128,0,//orange		8
                    255,153,255,//pink		9
                    51,153,255,	//			10	
                    153, 102, 51,//			11
                    128, 51, 153,//			12
                    153, 153, 51,//			13
                    163, 38, 51,//			14
                    204, 153, 102,//		15	
                    204, 224, 255,//		16
                    128, 179, 255,//		17
                    206, 255, 0,//			18
                    255,204,204,//			19
                    204,255,153,//			20

                 }; //20x3=60 color elements

bool is_number(const std::string& s){

  std::string::const_iterator it = s.begin();
      while (it != s.end() && std::isdigit(*it)) ++it;
      return !s.empty() && it == s.end();
  /*
    return !s.empty() && std::find_if(s.begin(),
        s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
        */
}                 

void readCloudFromFile(int argc, char** argv, std::vector<htr::Point3D>& points,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){

  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;

  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if(filenames.size()<=0){
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()<=0){
          filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
          if(filenames.size()<=0){
              filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
              if(filenames.size()<=0){
                  std::cerr << "Usage: ./dbscan <file.txt> <octree resolution> <eps> <minPtsAux> <minPts> <output dir> output extension = pcd(default)" << std::endl;
                  return std::exit(-1);
              }else if(filenames.size() == 1){
                  file_is_xyz = true;
              }
          }else if(filenames.size() == 1){
             file_is_txt = true;
        }
    }else if(filenames.size() == 1){
          file_is_pcd = true;
    }
  }
  else if(filenames.size() == 1){
      file_is_ply = true;
  }else{
      std::cerr << "Usage: ./dbscan <file.txt> <octree resolution> <eps> <minPtsAux> <minPts> <output dir> output extension = pcd(default)" << std::endl;
      return std::exit(-1);
  }

  if(file_is_pcd){ 
      if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0){
          std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
          std::cerr << "Usage: ./dbscan <file.txt> <octree resolution> <eps> <minPtsAux> <minPts> <output dir> output extension = pcd(default)" << std::endl;
          return std::exit(-1);
      }
      pcl::console::print_info("\nFound pcd file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_ply){
      pcl::io::loadPLYFile(argv[filenames[0]],*cloud);
      if(cloud->points.size()<=0 or cloud->points[0].x<=0 and cloud->points[0].y<=0 and cloud->points[0].z<=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *cloud);
          if(cloud->points.size()<=0 or cloud->points[0].x<=0 and cloud->points[0].y<=0 and cloud->points[0].z<=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*cloud);
              if(cloud->points.size()<=0 or cloud->points[0].x<=0 and cloud->points[0].y<=0 and cloud->points[0].z<=0){
                  pcl::console::print_error("\nError. ply file is not compatible.\n");
                  return std::exit(-1);
              }
          }
       }

      pcl::console::print_info("\nFound ply file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");

    }else if(file_is_txt){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return std::exit(-1);
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;
      unsigned int r, g, b; 

      while(file >> x_ >> y_ >> z_ >> r >> g >> b){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;            
          
          uint8_t r_, g_, b_; 
          r_ = uint8_t(r); 
          g_ = uint8_t(g); 
          b_ = uint8_t(b); 

          uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_); 
          pt.rgb = *reinterpret_cast<float*>(&rgb_);               
              
          cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->points.size ());
      pcl::console::print_info (" points]\n");
      
  }else if(file_is_xyz){
  
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return std::exit(-1);
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;

      while(file >> x_ >> y_ >> z_){
          
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;            
          
          cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound xyz file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->points.size ());
      pcl::console::print_info (" points]\n");
  }


  cloud->width = (int) cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;  
  
  for(int i =0; i<cloud->points.size();i++){
       htr::Point3D aux;
       aux.x = cloud->points[i].x;
       aux.y = cloud->points[i].y;
       aux.z = cloud->points[i].z;
       
       uint32_t rgb_ = *reinterpret_cast<int*>(&cloud->points[i].rgb); 
       uint8_t r_, g_, b_; 

       r_ = (rgb_ >> 16) & 0x0000ff; 
       g_ = (rgb_ >> 8)  & 0x0000ff; 
       b_ = (rgb_)       & 0x0000ff; 

       unsigned int r, g, b; 
       r = *((uint8_t *) &r_); 
       g = *((uint8_t *) &g_); 
       b = *((uint8_t *) &b_);   
       
       aux.r = r;
       aux.g = g;
       aux.b = b;          

       points.push_back(aux);  
       
  }

  //calculateCentroid(points);
}

void init(int argc, char** argv,bool show,std::string extension){

  std::vector<htr::Point3D> groupA;
  dbScanSpace::dbscan dbscan;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  readCloudFromFile(argc, argv, groupA,cloud);

  //----------------------------------------------------------------
  std::string octreeResolution_str = argv[2];
  std::string eps_str = argv[3];//40.0f 
  std::string minPtsAux_str = argv[4];//>=3    /*INPUT PARAMETERS*/    
  std::string minPts_str = argv[5];//>=3 
  std::string output_dir = argv[6];//10
  //----------------------------------------------------------------

  //----------------------------------------------------------------
  double a;

  if(not is_number(octreeResolution_str)){
       PCL_ERROR("\nError: enter a valid octree resolution\n");
       std::exit(-1);
  }else if(not is_number(eps_str) and not(std::fmod(std::atof(eps_str.c_str()), 1.0))){  
       PCL_ERROR("\nError: enter a valid epsilon\n");
       std::exit(-1);													/*VALIDATION*/	
  }else if(not is_number(minPtsAux_str)){
       PCL_ERROR("\nError: enter a valid min points aux\n");
       std::exit(-1);
  }else if(not is_number(minPts_str)){
       PCL_ERROR("\nError: enter a valid min points\n");
       std::exit(-1);
  }

  boost::filesystem::path dirPath(output_dir);			

  if(not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)){
	    pcl::console::print_error("\nError. does not exist or it's not valid: ");
	    std::cout << output_dir << std::endl;
  	    std::exit(-1);
  }

  //----------------------------------------------------------------

  int octreeResolution = std::atoi(octreeResolution_str.c_str());
  float eps = std::atof(eps_str.c_str());
  int minPtsAux_ = std::atoi(minPtsAux_str.c_str());
  int minPts = std::atoi(minPts_str.c_str()); 

  if(minPts < 3){
  	pcl::console::print_error("\nminPts must be >= 3! \n");
    std::exit(-1);
  }

  /*
  DBSCAN algorithm requires 3 parameters:
   - octreeResolution, describes the length of the smallest voxels at lowest octree level.
   - epsilon, which specifies how close points should be to each other to be considered a part of a cluster
   - minPts, which specifies how many neighbors a point should have to be included into a cluster (must be >=3).
  */

  //groupA.size()*0.001 -> eps (you can set this param for epsilon)
  //----------------------------------------------------------------
  dbscan.init(groupA, octreeResolution, eps, minPtsAux_, minPts); /*RUN DBSCAN*/
  //----------------------------------------------------------------

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  //dbscan.generateClusters();
  dbscan.generateClusters_fast();
  //dbscan.generateClusters_one_step();

  ofstream fout;
  int cont = 0;

  if(dbscan.getClusters().size()<=0){

  	  pcl::console::print_error("\nCould not generated clusters, bad parameters\n");
  	  std::exit(-1);

  }

  //-----------------Save cloud_cluster_#.txt:-------------------------//

  if(extension == "txt"){

	  for(auto& cluster : dbscan.getClusters()){

	      std::string str1 = output_dir;
	      str1 += "/cloud_cluster_";
	      str1 += std::to_string(cont);
	      str1 += ".txt";

	      fout.open(str1.c_str());

	      for(auto& point:cluster.clusterPoints){    

	        uint32_t rgb_ = *reinterpret_cast<int*>(&point.rgb); 
	        uint8_t r_, g_, b_; 

	        r_ = (rgb_ >> 16) & 0x0000ff; 
	        g_ = (rgb_ >> 8)  & 0x0000ff; 
	        b_ = (rgb_)       & 0x0000ff; 

	        unsigned int r, g, b; 
	        r = *((uint8_t *) &r_); 
	        g = *((uint8_t *) &g_); 
	        b = *((uint8_t *) &b_);      
	       
	        fout << point.x << " " << point.y << " "<< point.z << " " << r << " " << g << " " << b << std::endl;               
	  
	      }

	      fout.close();
	      cont +=1;
	    }

  /*ofstream fout2;
  std::string str2 = output_dir;
  str2 += "/clusters_number.txt";
  fout2.open(str2.c_str());
  fout2 << cont << std::endl;
  fout2.close();*/

  }else if(extension == "pcd"){

  	 for(auto& cluster : dbscan.getClusters()){

	      std::string str1 = output_dir;
	      str1 += "/cloud_cluster_";
	      str1 += std::to_string(cont);
	      str1 += ".pcd";

	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_pcd (new pcl::PointCloud<pcl::PointXYZRGB>());

	      for(auto& point:cluster.clusterPoints){   

		        pcl::PointXYZRGB pt;

		        pt.x = point.x;
		        pt.y = point.y;
		        pt.z = point.z; 

		        pt.r = point.r;
		        pt.g = point.g;
		        pt.b = point.b; 

		      	cloud_cluster_pcd->points.push_back(pt);        	  
	      }

	      pcl::io::savePCDFileBinary(str1.c_str(), *cloud_cluster_pcd);  
	      cont +=1;
	 }

  }else if(extension == "ply"){

  	 for(auto& cluster : dbscan.getClusters()){

	      std::string str1 = output_dir;
	      str1 += "/cloud_cluster_";
	      str1 += std::to_string(cont);
	      str1 += ".ply";

	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_ply (new pcl::PointCloud<pcl::PointXYZRGB>());

	      for(auto& point:cluster.clusterPoints){   

		        pcl::PointXYZRGB pt;

		        pt.x = point.x;
		        pt.y = point.y;
		        pt.z = point.z; 

		        pt.r = point.r;
		        pt.g = point.g;
		        pt.b = point.b; 

		      	cloud_cluster_ply->points.push_back(pt);        	  
	      }

	      pcl::PLYWriter writer;
 		  writer.write(str1.c_str(), *cloud_cluster_ply, false, false);
	      cont +=1;
	 }

  }else if(extension == "xyz"){

	  for(auto& cluster : dbscan.getClusters()){

	      std::string str1 = output_dir;
	      str1 += "/cloud_cluster_";
	      str1 += std::to_string(cont);
	      str1 += ".xyz";

	      fout.open(str1.c_str());

	      for(auto& point:cluster.clusterPoints){    
	       
	        fout << point.x << " " << point.y << " "<< point.z << std::endl;               
	  
	      }

	      fout.close();
	      cont +=1;
	    }
  }

  //-------------------------------------------------------------------//

  end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end-start;
  std::cout << "\nelapsed time: " << elapsed_seconds.count() << "s\n";

  //-----------------Visualize clusters pcl-visualizer-----------------//

  if(show){

      //std::cout << "\nPrinting clusters..." << std::endl;

      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("DBSCAN CLUSTERS"));

      int PORT1 = 0;
      viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
      viewer->setBackgroundColor (0, 0, 0, PORT1);
      viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

      int PORT2 = 0;
      viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
      viewer->setBackgroundColor (0, 0, 0, PORT2);
      viewer->addText("SEGMENTATION", 10, 10, "PORT2", PORT2);

      viewer->setPosition(0,0);
      viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark

      int numClust = 0;
      //viewer->addPointCloud(cloud,"Original_Cloud",PORT1);

      std::random_device seeder;
      std::ranlux48 gen(seeder());
      std::uniform_int_distribution<int>  uniform_0_255(0, 255);
      
      int j = 0;

      for(auto& cluster : dbscan.getClusters()){

          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb (new pcl::PointCloud<pcl::PointXYZRGB>());
          //uint8_t r(255), g(15), b(15); 
          
          uint8_t r;
          uint8_t g;
          uint8_t b; 
          
          if(j < 60){
          
            r = (uint8_t) colors[j];
            g = (uint8_t) colors[j+1];
            b = (uint8_t) colors[j+2];
                    
          }else{
            
            r = (uint8_t) uniform_0_255(gen);
            g = (uint8_t) uniform_0_255(gen);
            b = (uint8_t) uniform_0_255(gen); 
                     
          }  

          //Adding different color to each cluster    
   
          for(auto& pointCluster:cluster.clusterPoints){

              pcl::PointXYZRGB point;
              point.x = pointCluster.x;
              point.y = pointCluster.y;
              point.z = pointCluster.z;
              
              uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
              point.rgb = *reinterpret_cast<float*>(&rgb);

              cluster_rgb->points.push_back(point);
          }
          
          j+=3;

          std::string nameId = "cluster_";
          nameId += std::to_string(numClust);

          //std::cout << "Adding: " << nameId << " to pcl visualizer" << std::endl;
          viewer->addPointCloud(cluster_rgb,nameId.c_str(),PORT2);
          numClust += 1;
      }
      
      double scale = 1;
      
      viewer->addCoordinateSystem(scale);
      pcl::PointXYZ p11, p22, p33;
      p11.getArray3fMap() << 1, 0, 0;
      p22.getArray3fMap() << 0, 1, 0;
      p33.getArray3fMap() << 0,0.1,1;

      viewer->addText3D("x", p11, 0.2, 1, 0, 0, "x_");
      viewer->addText3D("y", p22, 0.2, 0, 1, 0, "y_");
      viewer->addText3D ("z", p33, 0.2, 0, 0, 1, "z_");
      
      if(cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b<= 0 ){
         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud,255,255,0);
         viewer->addPointCloud(cloud,color_handler,"Original_Cloud",PORT1);
      }else{
         viewer->addPointCloud(cloud,"Original_Cloud",PORT1);
      }
  

      viewer->initCameraParameters();
      viewer->resetCamera();
      
      //std::cout << "\nGenerated: " << numClust << " clusters" << std::endl;
      pcl::console::print_info("\nGenerated: ");
      pcl::console::print_value ("%d", numClust);
      pcl::console::print_info (" clusters\n");

      pcl::console::print_info ("\npress [q] to exit!\n");

      while(!viewer->wasStopped()){
          viewer->spin();
      }
   }
}


int main(int argc, char** argv){

   if(argc < 7 or argc > 8){
   
      std::cerr << "Usage: ./dbscan <file.ply> <octree resolution> <eps> <minPts aux> <minPts> <output dir> <output extension = pcd(default)>" << std::endl;
      std::cerr << "Support: ply - pcd - txt - xyz" << std::endl;
      return -1;
   }

   std::string extension;

   if(argc == 7){
       extension = "pcd";
   }else{

	   	std::string temp = argv[7];
	  
	  	if(temp == "ply"){
	  		extension = "ply";
	    }else if(temp == "txt"){
	    	extension = "txt";
	    }else if(temp == "xyz"){
	    	extension = "xyz";
	    }else{
	    	std::cout << "output extension must be: ply, txt or xyz (default=pcd)" << std::endl;
	  		std::exit(-1);
	    }	  		
  	}

std::cout << "\n*************************************" << std::endl;
std::cout << "*** DBSCAN Cluster Segmentation *** " << std::endl;
std::cout << "*************************************" << std::endl;

bool showClusters = true;

init(argc,argv,showClusters,extension);

return 0;

}
