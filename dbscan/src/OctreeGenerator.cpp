#include <dbscan/OctreeGenerator.h>

namespace htr {

/// The default constructor.
OctreeGenerator::OctreeGenerator()
    : cloud(new CloudXYZ),
      octree_p(new OctreeXYZSearch(20)),
      // octree(56),
      currentExtractionLevel(0) {}

/// The default destructor.
OctreeGenerator::~OctreeGenerator() {}

/// Initializes pcl's cloud data structure with random values centered at 0,0,0.
///@param[in] width The width of the point cloud.
///@param[in] height The height of the point cloud.
///@param[in] depth The depth of the point cloud.
///@param[in] numOfPoints The num of points in the point cloud.
void OctreeGenerator::initRandomCloud(const float width, const float height, const float depth, const int numOfPoints) {
  srand((unsigned int)time(NULL));
  // Generate pointcloud data

  cloud->width = numOfPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = (width * rand() / ((float)RAND_MAX + 1.0f)) - width / 2;
    cloud->points[i].y = (height * rand() / ((float)RAND_MAX + 1.0f)) - height / 2;
    cloud->points[i].z = (depth * rand() / ((float)RAND_MAX + 1.0f)) - depth / 2;
  }
}

/// Calculates the entire cloud centroid
void OctreeGenerator::calculateCloudCentroid() {
  Eigen::Matrix<float, 4, 1> centroid_original;

  pcl::compute3DCentroid(*cloud, centroid_original);
  cloudCentroid.x = centroid_original[0];
  cloudCentroid.y = centroid_original[1];
  cloudCentroid.z = centroid_original[2];
}

/// Initializes the octree from the cloud data provided at the specified resolution.
///@param[in] resolution The voxel edge size at the minimum subdivision level.
void OctreeGenerator::initOctree(const int resolution) {
  //************************************
  // octree_p.reset(new OctreeXYZSearch(resolution));
  octree_p->deleteTree();
  octree_p->setResolution(resolution);

  // octree_p->setInputCloud(cloud);
  octree_p->setInputCloud(cloud);
  octree_p->addPointsFromInputCloud();

  currentExtractionLevel = octree_p->getTreeDepth();
  extractPointsAtLevel(currentExtractionLevel);
  //************************************
  // octree.setInputCloud (cloud);
  // octree.addPointsFromInputCloud ();
  // octree.setResolution(resolution);
  // currentExtractionLevel = octree.getTreeDepth();
  // extractPointsAtLevel(currentExtractionLevel);
}

/// Calculates the position of each voxel that exists at a specified tree depth.
///@param[in] depth The selected tree depth in the octree.
void OctreeGenerator::extractPointsAtLevel(const int depth) {
  if (depth >= 0 && depth <= octree_p->getTreeDepth()) {
    currentExtractionLevel = depth;

    OctreeXYZSearch::Iterator tree_it;
    OctreeXYZSearch::Iterator tree_it_end = octree_p->end();

    pcl::PointXYZRGB pt;
    // cout << "===== Extracting data at depth " << depth << "... " << endl;
    // double start = pcl::getTime ();

    octreeVoxels.clear();
    octreeCentroids.clear();

    // Check if end iterator can be substituted for the corresponding level so
    // further level checking is avoided
    for (tree_it = octree_p->begin(depth); tree_it != tree_it_end; ++tree_it) {
      // Level check, discards all nodes that do not belong to desired level
      if (tree_it.getCurrentOctreeDepth() == depth) {
        Eigen::Vector3f voxel_min, voxel_max;

        octree_p->getVoxelBounds(tree_it, voxel_min, voxel_max);

        // Get voxel center point
        Point3D p = {(voxel_min.x() + voxel_max.x()) / 2.0f, (voxel_min.y() + voxel_max.y()) / 2.0f, (voxel_min.z() + voxel_max.z()) / 2.0f};
        Voxel v = {p, voxel_max.x() - voxel_min.x()};
        // TODO: remove redundant info
        octreeVoxels.push_back(v);
        octreeCentroids.push_back(p);
      }
    }
    // cout<<"Extracted Points: "<<displayPoints.size()<<endl;
    // double end = pcl::getTime ();
    // printf("%zu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
    //(end - start) / static_cast<double> (displayCloud->points.size ()));
  }
}

///@deprecated
void OctreeGenerator::stepExtractionLevel(const int step) { extractPointsAtLevel(currentExtractionLevel + step); }
}  // namespace htr