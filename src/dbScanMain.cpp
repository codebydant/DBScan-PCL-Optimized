#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <algorithm>
#include <random>
#include <vector>
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
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

void readCloudFromFile(const char* filename, std::vector<htr::Point3D>& points){

    pcl::console::TicToc tt;
    std::cout << "Reading cloud from file...";
 
    double x, y, z; 
    std::ifstream file(filename);
      
    if(!file.is_open()){    
      std::cerr << "Error: Could not find " << filename << std::endl;        
      return std::exit(-1);    
    }  
         
    while(file >> x >> y >> z){
       htr::Point3D aux;
       aux.x = x;
       aux.y = y;
       aux.z = z;
       points.push_back(aux);
       
    }
    
    pcl::console::print_info ("[done, ");
    pcl::console::print_value ("%g", tt.toc ());
    pcl::console::print_info (" ms : ");
    pcl::console::print_value ("%d", points.size());
    pcl::console::print_info (" points]\n\n");
  
    file.close();   
    calculateCentroid(points);
}

void init(char** argv,bool show){

    readCloudFromFile(argv[1], groupA);

	float eps = std::atof(argv[2]); //40.0f
	int minPts = std::atof(argv[3]); //10
	int maxPts = std::atof(argv[4]); //1000
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
		    
        std::string str1 = "cloud";
        str1 += "_cluster_";
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
     std::string str2 = "clusters_number.txt";   
     fout2.open(str2.c_str());  
     fout2 << cont << std::endl;
     fout2.close();   

     end = std::chrono::system_clock::now();

     std::chrono::duration<double> elapsed_seconds = end-start;
     std::cout << "\nelapsed time: " << elapsed_seconds.count() << "s\n";
     
     if(show){
     
         std::cout << "\nPrinting clusters..." << std::endl;
         
         boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("DBSCAN CLUSTERS"));

         viewer->setPosition(0,0);
         viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark 
         
         int numClust = 0;
         
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
            viewer->addPointCloud(cluster_rgb,nameId.c_str());        
            numClust += 1;                                             
        }  

        viewer->initCameraParameters();
        viewer->resetCamera();
       
        pcl::console::print_info ("\npress [q] to exit!\n");    
        
        while(!viewer->wasStopped()){
           viewer->spin();
        } 
             
     }           	 
}


int main(int argc, char** argv){

   if(argc < 5 or argc > 5){
   
      std::cerr << "Usage: ./dbscan <file.txt> <eps> <minPts> <maxPts>" << std::endl;
      return -1;
   }
   
   std::cout << "\n*************************************" << std::endl;
   std::cout << "*** DBSCAN Cluster Segmentation *** " << std::endl;
   std::cout << "*************************************" << std::endl;
   
   bool showClusters = true;

   init(argv,showClusters);

   return 0;
}
