#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <algorithm>
#include <random>
#include <vector>
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply/ply.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>

#include "OctreeGenerator.h"
#include "mouseUtils.h"
#include "dbScan.h"
#include "HTRBasicDataStructures.h"

using namespace std;

vector<htr::Point3D> groupA;
pcl::PointXYZ centroid;

dbScanSpace::dbscan *dbscan, dbscan2;


void calculateCentroid(vector<htr::Point3D>& points)
{
    for(htr::Point3D point:points)
    {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    centroid.z /= points.size();
}

void readCloudFromFile(int argc, char** argv, std::vector<htr::Point3D>& points,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){


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
                  std::cerr << "Usage: ./dbscan <file.txt> <eps> <minPts> <maxPts> <output dir>" << std::endl;
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
      std::cerr << "Usage: ./dbscan <file.txt> <eps> <minPts> <maxPts> <output dir>" << std::endl;
      return std::exit(-1);
  }

  if(file_is_pcd){ 
      if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0){
          std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
          std::cerr << "Usage: ./dbscan <file.txt> <eps> <minPts> <maxPts> <output dir>" << std::endl;
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
      if(cloud->points.size()<=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *cloud);
          if(cloud->points.size()<=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*cloud);
              if(cloud->points.size()<=0){
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

    }else if(file_is_txt or file_is_xyz){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return std::exit(-1);
      }
      double x_,y_,z_;
      while(file >> x_ >> y_ >> z_){
          pcl::PointXYZ pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;
          cloud->points.push_back(pt);
      }

      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
  }

  cloud->width = (int) cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;  
  
  for(int i =0; i<cloud->points.size();i++){
       htr::Point3D aux;
       aux.x = cloud->points[i].x;
       aux.y = cloud->points[i].y;;
       aux.z = cloud->points[i].z;
       points.push_back(aux);  
  }

  calculateCentroid(points);
}

void init(int argc, char** argv,bool show){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    readCloudFromFile(argc, argv, groupA,cloud);

	float eps = std::atof(argv[2]); //40.0f
	int minPts = std::atof(argv[3]); //10
	int maxPts = std::atof(argv[4]); //1000
	std::string output_dir = argv[5];
    dbscan2.init(groupA, eps, eps, minPts, maxPts);
    
    /*
    DBSCAN algorithm requires 2 parameters - epsilon , which specifies how close points should be to each other to be considered
    a part of a cluster; and minPts , which specifies how many neighbors a point should have to be included into a cluster. 
    However, you may not know these values in advance.
    */    

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

	dbscan2.generateClusters();
    //dbscan2.generateClusters_fast();
	//dbscan2.generateClusters_one_step();
	
	ofstream fout;
    int cont = 0;   

	for(auto& cluster : dbscan2.getClusters()){
		
        std::cout << "cluster " << cont << " size " << cluster.clusterPoints.size() << endl;
		    
        std::string str1 = output_dir;
        str1 += "/cloud_cluster_";
        str1 += to_string(cont);
        str1 += ".xyz";

        fout.open(str1.c_str());            
            
        for(auto& point:cluster.clusterPoints){
           
            fout << point.x << " " << point.y << " "<< point.z << endl;            
        }
        
        fout.close();
        cont +=1;   
     }	
      
     ofstream fout2;      
     std::string str2 = output_dir; 
     str2 += "/clusters_number.txt";   
     fout2.open(str2.c_str());  
     fout2 << cont << std::endl;
     fout2.close();   

     end = std::chrono::system_clock::now();

     std::chrono::duration<double> elapsed_seconds = end-start;
     std::cout << "\nelapsed time: " << elapsed_seconds.count() << "s\n";
     
     if(show){
     
         std::cout << "\nPrinting clusters..." << std::endl;
         
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
         viewer->addPointCloud(cloud,"Original_Cloud",PORT1);
         
         for(auto& cluster : dbscan2.getClusters()){       
         
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb (new pcl::PointCloud<pcl::PointXYZRGB>());
            //uint8_t r(255), g(15), b(15); 
            uint8_t r = (uint8_t) rand();
            uint8_t g = (uint8_t) rand();
            uint8_t b = (uint8_t) rand();        
                        
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
            
            std::string nameId = "cluster_";
            nameId += std::to_string(numClust);
            
            std::cout << "Adding: " << nameId << " to pcl visualizer" << std::endl;        
            viewer->addPointCloud(cluster_rgb,nameId.c_str(),PORT2);
            numClust += 1;                                             
        }  

        pcl::PointXYZ p11, p22, p33;
        p11.getArray3fMap() << 1, 0, 0;
        p22.getArray3fMap() << 0, 1, 0;
        p33.getArray3fMap() << 0,0.1,1;

        viewer->addText3D("x", p11, 0.2, 1, 0, 0, "x_");
        viewer->addText3D("y", p22, 0.2, 0, 1, 0, "y_");
        viewer->addText3D ("z", p33, 0.2, 0, 0, 1, "z_");

        viewer->initCameraParameters();
        viewer->resetCamera();
       
        pcl::console::print_info ("\npress [q] to exit!\n");    
        
        while(!viewer->wasStopped()){
           viewer->spin();
        } 
             
     }           	 
}


int main(int argc, char** argv){

   if(argc < 6 or argc > 6){
   
      std::cerr << "Usage: ./dbscan <file.txt> <eps> <minPts> <maxPts> <output dir>" << std::endl;
      return -1;
   }
   
   std::cout << "\n*************************************" << std::endl;
   std::cout << "*** DBSCAN Cluster Segmentation *** " << std::endl;
   std::cout << "*************************************" << std::endl;
   
   bool showClusters = true;

   init(argc,argv,showClusters);

   return 0;
}
