//
// Created by florian on 09/07/20.
//

#pragma once
#include <Eigen/Eigen>

#ifndef DBSCAN_KEYPOINTCLUSTER_H
#define DBSCAN_KEYPOINTCLUSTER_H


class KeypointCluster{
public:
    int ClusterID;
    int ClusterSize;
    //Eigen::Matrix3f& Eigenvectors;
    //Eigen::Vector3f& Eigenvalues;
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
    //void set_values(int,int,Eigen::Matrix3f&, Eigen::Vector3f&, float,float,float,float,float,float);
    void set_values(int,int, float,float,float,float,float,float);

};


#endif //DBSCAN_KEYPOINTCLUSTER_H
