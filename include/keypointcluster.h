//
// Created by florian on 09/07/20.
//

#pragma once

#ifndef DBSCAN_KEYPOINTCLUSTER_H
#define DBSCAN_KEYPOINTCLUSTER_H

#include <fstream>
#include <iostream>
#include <regex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/pca.h>

#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace Eigen;
typedef std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> PointsT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGBT;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudRGBT_Ptr;
typedef pcl::PointXYZ PointT;
typedef pcl::PCA<pcl::PointXYZRGB> PCA;
typedef pcl::PointXYZRGB PointRGBT;
typedef std::vector<Vector3d, aligned_allocator<Vector3d>> corners;
typedef std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> > covs;




class KeypointCluster{
private:

public:
    int ClusterID;
    int ClusterSize;
    PointT Centroid;
    Eigen::Matrix3f Eigenvectors;
    Eigen::Vector3f Eigenvalues;
    PointCloudRGBT key_cloud;
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;

    void set_cloud(int ClusterID, PointCloudRGBT& key_cloud,PointT);
    void set_values(int,Eigen::Matrix3f, Eigen::Vector3f, float,float,float,float,float,float);

};


#endif //DBSCAN_KEYPOINTCLUSTER_H


KeypointCluster calculate_cluster_descriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr KeyCloud, KeypointCluster KeyCluster);
void add_cluster_visu(pcl::visualization::PCLVisualizer::Ptr viewer,  KeypointCluster KeyCluster);