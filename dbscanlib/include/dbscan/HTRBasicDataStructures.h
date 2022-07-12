/**
 *@file HTRBasicDataStructures.h
 *Data structures that do not depend on external classes.
 * https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html#adding-your-own-custom-pointt-type
 */

#ifndef HTR_BASIC_DATA_STRUCTURES_H
#define HTR_BASIC_DATA_STRUCTURES_H
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

/// Modified pcl point to include an id.
namespace pcl {

class mod_pointXYZ : public PointXYZRGB {
 public:
  mod_pointXYZ() {
    x = y = z = 0;
    id = 0;
  }

  mod_pointXYZ(float x, float y, float z) : PointXYZRGB(x, y, z) { id = 0; }
  /*
  mod_pointXYZ(unsigned int r, unsigned int g, unsigned int b) : PointXYZRGB(r,g,b){
      id = 0;
  }
   */
  int id;
};
}  // namespace pcl

namespace htr {
struct Index2D {
  int x;
  int y;
  // Index2D():x(0),y(0){}
};

struct Point3D {
  float x;
  float y;
  float z;

  unsigned int r;
  unsigned int g;
  unsigned int b;

  void initRandom() {
    x = (rand() % 40);
    y = (rand() % 40);
    z = (rand() % 40);
  }
};

struct FlaggedPoint3D {
  Point3D point;
  int flag;
};

struct DepthPixel {
  int x;
  int y;
  float z;
};

struct LabeledPoint {
  Point3D point;
  int label;
};

struct CubeBoundary {
  Point3D start;
  Point3D end;
};

struct LinearBoundary {
  float start;
  float end;
};
}  // namespace htr

#endif
