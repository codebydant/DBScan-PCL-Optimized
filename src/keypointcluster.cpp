//
// Created by florian on 09/07/20.
//

#include <stdlib.h>
#include <iostream>
#include "keypointcluster.h"

//void KeypointCluster::set_values (int x_id, int y_size,Eigen::Matrix3f& e_vecs,Eigen::Vector3f& e_vals, float a, float b, float c, float d, float e, float f) {
void KeypointCluster::set_values (int x_id, int y_size, float a, float b, float c, float d, float e, float f) {

    ClusterID=x_id;
    ClusterSize=y_size;
    //Eigenvectors=e_vecs;
    //Eigenvalues=e_vals;
    minX = a;
    minY = b;
    minZ = c;
    maxY = d;
    maxX = e;
    maxZ = f;
}
