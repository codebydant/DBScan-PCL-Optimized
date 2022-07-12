/**
 *@class OctreeGenerator
 *Creates an octree from the point cloud data provided.
 *
 */

#ifndef OCTREE_GENERATOR_H
#define OCTREE_GENERATOR_H
// #include <pcl/octree/octree_impl.h>

#include <pcl/common/centroid.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <vector>

#include "HTRBasicDataStructures.h"

namespace htr {

class OctreeGenerator {
 public:
  typedef pcl::PointCloud<pcl::mod_pointXYZ> CloudXYZ;
  typedef pcl::octree::OctreePointCloudSearch<pcl::mod_pointXYZ> OctreeXYZSearch;
  typedef pcl::octree::OctreePointCloudSearch<pcl::mod_pointXYZ>::LeafNode LeafNode;
  typedef pcl::octree::OctreePointCloudSearch<pcl::mod_pointXYZ>::LeafNodeIterator LeafNodeIterator;
  typedef htr::Point3D Point3D;

  struct Voxel {
    Point3D position;
    float size;
  };

  OctreeGenerator();
  ~OctreeGenerator();

/// This is needed to remove the warning about alignment: object allocated on the heap may not be
/// aligned 16
/// Solution specific to microsoft.
#ifdef _MSC_VER
  void *operator new(size_t size) {
    void *p = _aligned_malloc(size, 16);
    if (!p) throw std::bad_alloc();
    return p;
  }

  void operator delete(void *p) {
    OctreeGenerator *ptr = static_cast<OctreeGenerator *>(p);
    _aligned_free(p);
  }
#endif
  inline CloudXYZ::Ptr getCloud() { return cloud; }
  inline pcl::mod_pointXYZ getCloudCentroid() { return cloudCentroid; }
  inline std::vector<Voxel> &getVoxels() { return octreeVoxels; }
  inline std::vector<Point3D> &getCentroids() { return octreeCentroids; }
  inline OctreeXYZSearch::Ptr getOctree() { return octree_p; }

  void initRandomCloud(const float width, const float height, const float depth, const int numOfPoints);

  template <typename T>
  void initCloudFromVector(const std::vector<T> &points, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);

  void initOctree(const int resolution);

  void extractPointsAtLevel(const int depth);
  void stepExtractionLevel(const int step);

 private:
  unsigned int currentExtractionLevel;
  pcl::PointCloud<pcl::mod_pointXYZ>::Ptr cloud;
  OctreeXYZSearch::Ptr octree_p;

  pcl::mod_pointXYZ cloudCentroid;

  std::vector<Voxel> octreeVoxels;
  std::vector<Point3D> octreeCentroids;

  void calculateCloudCentroid();
};

/// Initializes pcl's cloud data structure from a vector of any type containing x, y, and z member variables.
///@param[in] points The input data vector.
template <typename T>
void OctreeGenerator::initCloudFromVector(const std::vector<T> &points, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud) {
  // Note: Width and Height are only used to store the cloud as an image.
  // Source width and height can be used instead of a linear representation.
  cloud->width = input_cloud->points.size();
  cloud->height = 1;

  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = input_cloud->points[i].x;
    cloud->points[i].y = input_cloud->points[i].y;
    cloud->points[i].z = input_cloud->points[i].z;

    uint8_t r_, g_, b_;
    r_ = uint8_t(input_cloud->points[i].r);
    g_ = uint8_t(input_cloud->points[i].g);
    b_ = uint8_t(input_cloud->points[i].b);

    uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_);
    cloud->points[i].rgb = *reinterpret_cast<float *>(&rgb_);
  }
  calculateCloudCentroid();
}
}  // namespace htr

#endif
