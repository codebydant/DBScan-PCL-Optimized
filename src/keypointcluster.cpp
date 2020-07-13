//
// Created by florian on 09/07/20.
//

#include <stdlib.h>
#include "keypointcluster.h"
#include <Eigen/Core>

void KeypointCluster::set_cloud(int ClusterID_int, PointCloudRGBT& key_cloud_c, PointT Centroid_p){
    ClusterID=ClusterID_int;
    Centroid=Centroid_p;
    key_cloud=key_cloud_c;
};


void KeypointCluster::set_values ( int y_size,Eigen::Matrix3f e_vecs,Eigen::Vector3f e_vals, float a, float b, float c, float d, float e, float f) {
    //Centroid=Centroid_p;
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


KeypointCluster calculate_cluster_descriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Keycloud, KeypointCluster KeyCluster){
    std::cout << "Start testing function: " <<std::endl;
    pcl::PointXYZRGB minPt, maxPt;

    pcl::getMinMax3D (*Keycloud, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;


    //Source: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*Keycloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*Keycloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCA<pcl::PointXYZRGB> pca;
    pca.setInputCloud(Keycloud);
    pca.project(*Keycloud, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

    KeyCluster.set_values(Keycloud->size(),pca.getEigenVectors(),pca.getEigenValues(),minPt.x, minPt.y, minPt.z,maxPt.x, maxPt.y, maxPt.z);
    return KeyCluster;
}




/*
pcl::PointXYZRGB minPoint, maxPoint;
pcl::getMinMax3D(*cloud_cluster_pcd, minPoint, maxPoint);
const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
// Final transform
const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
*/
void add_cluster_visu(pcl::visualization::PCLVisualizer::Ptr viewer,  KeypointCluster KeyCluster){
    string lineID = to_string(KeyCluster.ClusterID);
    pcl::PointXYZ src_idx, tgt_idx;
    src_idx= KeyCluster.Centroid;

    std::cout <<"eigenvect:"<<KeyCluster.Eigenvectors(0,0)<< std::endl;
    std::cout <<"eigenvect:"<<KeyCluster.Eigenvectors(1,0)<< std::endl;
    std::cout <<"eigenvect:"<<KeyCluster.Eigenvectors(2,0)<< std::endl;

    std::cout <<"eigenvalue:"<<KeyCluster.Eigenvalues[0]<< std::endl;
    std::cout <<"eigenvalue:"<<KeyCluster.Eigenvalues[1]<< std::endl;
    std::cout <<"eigenvalue:"<<KeyCluster.Eigenvalues[2]<< std::endl;

    float scaller, offset;
    int visu_scale=50, vec_cnt=0;
    scaller =(KeyCluster.Eigenvalues[0]+KeyCluster.Eigenvalues[1]+KeyCluster.Eigenvalues[2]);
    tgt_idx=src_idx;
    offset=visu_scale*(KeyCluster.Eigenvalues[vec_cnt]/scaller);

    tgt_idx.x=src_idx.x+offset*KeyCluster.Eigenvectors(vec_cnt,0);
    tgt_idx.y=src_idx.y+offset*KeyCluster.Eigenvectors(vec_cnt,1);
    tgt_idx.z=src_idx.z+offset*KeyCluster.Eigenvectors(vec_cnt,2);

    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 125, 125, 125, "x"+lineID);

    vec_cnt++;
    tgt_idx=src_idx;
    offset=visu_scale*(KeyCluster.Eigenvalues[vec_cnt]/scaller);

    tgt_idx.x=src_idx.x+offset*KeyCluster.Eigenvectors(0,vec_cnt);
    tgt_idx.y=src_idx.y+offset*KeyCluster.Eigenvectors(1,vec_cnt);
    tgt_idx.z=src_idx.z+offset*KeyCluster.Eigenvectors(2,vec_cnt);
    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 125, 125, 125, "y"+lineID);

    vec_cnt++;
    tgt_idx=src_idx;
    offset=visu_scale*(KeyCluster.Eigenvalues[vec_cnt]/scaller);

    tgt_idx.x=src_idx.x+offset*KeyCluster.Eigenvectors(0,vec_cnt);
    tgt_idx.y=src_idx.y+offset*KeyCluster.Eigenvectors(1,vec_cnt);
    tgt_idx.z=src_idx.z+offset*KeyCluster.Eigenvectors(2,vec_cnt);
    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 125, 125, 125, "z"+lineID);
};
