// the calculation method is based on this post
// https://towardsdatascience.com/machine-learning-clustering-dbscan-determine-the-optimal-value-for-epsilon-eps-python-example-3100091cfbc#:~:text=In%20layman's%20terms%2C%20we%20find,and%20select%20that%20as%20epsilon.
#include <bits/stdc++.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_plotter.h>
#include <vtkAutoInit.h>

#include <iostream>
#include <vector>
VTK_MODULE_INIT(vtkRenderingContextOpenGL2)
// #include "dbscan/HTRBasicDataStructures.h"
// #include "dbscan/OctreeGenerator.h"
/*************************************************************************************************/
void calculate_epsilon(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  // K nearest neighbor search
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  // ... populate the cloud and the search point
  // create a kd-tree instance
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // assign a point cloud - this builds the tree
  kdtree.setInputCloud(cloud_xyz);
  kdtree.setSortedResults(true);

  // pre-allocate the neighbor index and
  // distance vectors
  int K = 10;
  int cont = 0;
  std::vector<std::vector<float>> distances;
  for (size_t nIndex = 0; nIndex < cloud_xyz->points.size(); nIndex++) {
    pcl::PointXYZ searchPoint = cloud_xyz->points[nIndex];
    std::vector<int> pointsIdx(K);
    std::vector<float> pointsSquaredDist(K);

    // K nearest neighbor search
    kdtree.nearestKSearch(searchPoint, K, pointsIdx, pointsSquaredDist);

    // std::cout << "\nK nearest neighbor search at (" << searchPoint.x << " " << searchPoint.y << " "
    //           << searchPoint.z << ") with K=" << K << std::endl;
    cont += 1;
    if (cont == 100) {
      break;
    }

    distances.push_back(pointsSquaredDist);
  }

  std::vector<double> doubleVec_Y;
  for (size_t x = 0; x < distances.size(); x++) {
    for (double dist : distances[x]) {
      // for (int y = 0; y < distances[x]; y++) {
      doubleVec_Y.push_back((double)dist);
    }
  }

  double min_val = *std::min_element(doubleVec_Y.begin(), doubleVec_Y.end());
  double max_val = *std::max_element(doubleVec_Y.begin(), doubleVec_Y.end());

  std::vector<double> doubleVec_normalized;

  for (double x : doubleVec_Y) {
    double norm = ((x - min_val) / (max_val - min_val));
    doubleVec_normalized.push_back(norm);
  }

  sort(doubleVec_normalized.begin(), doubleVec_normalized.end());
  // std::partial_sort(doubleVec_normalized.begin(), doubleVec_normalized.begin() + 2, doubleVec_normalized.end());
  std::cout << "\nSorted squared distances (normalized): \n";
  // for (auto x : doubleVec_normalized) std::cout << x << " " << std::flush;

  std::vector<double> doubleVec_X;
  double cont_x = 0;
  for (size_t x = 0; x < doubleVec_normalized.size(); x++) {
    doubleVec_X.push_back(cont_x);
    cont_x += 1;
  }

  std::cout << "distance points: " << doubleVec_normalized.size() << std::endl;
  // https://pointclouds.org/documentation/classpcl_1_1visualization_1_1_p_c_l_plotter.html#ad38634e017541eb0df59bfc122cc9c3d
  pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter;
  plotter->addPlotData(doubleVec_X, doubleVec_normalized, "k square distance", vtkChart::POINTS, std::vector<char>());
  plotter->spinOnce(300);

  // -----------------------------------------------------------------------------------------
  // if (kdtree.nearestKSearch(searchPoint, K, pointsIdx, pointsSquaredDist) > 0) {
  //   for (std::size_t i = 0; i < pointsIdx.size(); ++i)
  //     std::cout << "    " << cloud->points[pointsIdx[i]].x << " " << cloud->points[pointsIdx[i]].y << " "
  //               << cloud->points[pointsIdx[i]].z << " (squared distance: " << pointsSquaredDist[i] << ")"
  //               << std::endl;
  // }

  // std::vector<double> doubleVec(pointsSquaredDist.begin(), pointsSquaredDist.end());

  // double min_val = *std::min_element(doubleVec.begin(), doubleVec.end());
  // double max_val = *std::max_element(doubleVec.begin(), doubleVec.end());

  // std::vector<double> doubleVec_normalized;

  // for (double x : doubleVec) {
  //   double norm = ((x - min_val) / (max_val - min_val));
  //   doubleVec_normalized.push_back(norm);
  // }

  // std::partial_sort(doubleVec_normalized.begin(), doubleVec_normalized.begin() + 2, doubleVec_normalized.end());
  // std::cout << "\nSorted squared distances (normalized): \n";

  // for (auto x : doubleVec_normalized) std::cout << x << " " << std::flush;

  // std::vector<double> doubleVec_X;
  // double cont_x = 0;

  // for (int x = 0; x < 110; x++) {
  //   doubleVec_X.push_back(cont_x);
  //   cont_x += 1;
  // }

  // // defining a plotter
  // pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter;
  // // adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
  // // plotter->addPlotData(doubleVec_normalized, doubleVec_X, "k square distance", vtkChart::LINE,
  // //                      std::vector<char>());
  // plotter->addPlotData(doubleVec_X, doubleVec_normalized, "k square distance", vtkChart::LINE,
  //                      std::vector<char>());

  // // display the plot, DONE!
  // // plotter->plot();
  // plotter->spinOnce(300);
}
