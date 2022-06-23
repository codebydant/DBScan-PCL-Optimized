/*
 * Abstract class for interface parses
 *
 * it implements an abstract class to be used by concrete classes. This module will
 * a common blueprint to the parser format.
 *
 * load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) must be
 * defined in the derived class to implement the new parser format extension.
 *
 * Be aware that this interface is being used in a factory class to provide supported parser
 * extensions to the client and is not meant to be used as an instance object
 */
#pragma once
#ifndef INTERFACE_PARSER_HPP
#define INTERFACE_PARSER_HPP
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <map>
#include <string>

namespace CloudParserLibrary {

class InterfaceParser {
 public:
  bool cloud_is_good(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if ((cloud->points.size() > 0) or (cloud->points[0].x > 0 && cloud->points[0].y > 0 && cloud->points[0].z > 0)) {
      return true;
    }
    return false;
  }
  virtual void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) = 0;
};

}  // namespace CloudParserLibrary
#endif