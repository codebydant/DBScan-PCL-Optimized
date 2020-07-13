//
// Created by florian on 09/07/20.
//

#include <stdlib.h>
#include "keypointcluster.h"
#include <Eigen/Core>

void KeypointCluster::set_cloud(int ClusterID, PointCloudRGBT& key_cloud){

};


void KeypointCluster::set_values ( int y_size,PointT Centroid_p,Eigen::Matrix3f e_vecs,Eigen::Vector3f e_vals, float a, float b, float c, float d, float e, float f) {
    Centroid=Centroid_p;
    ClusterSize=y_size;
    Eigenvectors=e_vecs;
    Eigenvalues=e_vals;
    minX = a;
    minY = b;
    minZ = c;
    maxY = d;
    maxX = e;
    maxZ = f;
}
