/*
 * Concrete parsers module
 *
 * it implements a factory to create a parser from a abstract class.
 * This module will provided support for new format extensions.
 */
#pragma once
#ifndef CONCRETE_PARSES_HPP
#define CONCRETE_PARSES_HPP
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <fstream>

#include "inteface_parser.hpp"

namespace CloudParserLibrary {
class ParserPCD : public InterfaceParser {
 public:
  std::string parser_name = "ParserPCD";
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (pcl::io::loadPCDFile(filename, *cloud) < 0) {
      pcl::console::print_error("Error loading point cloud %s \n", filename.c_str());
      std::exit(-1);
    }
  }
};

class ParserPLY : public InterfaceParser {
 public:
  std::string parser_name = "ParserPLY";
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    pcl::io::loadPLYFile(filename, *cloud);
    if (cloud_is_good(cloud)) {
      return;
    }

    pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");

    pcl::PolygonMesh cl;
    pcl::io::loadPolygonFile(filename, cl);
    pcl::fromPCLPointCloud2(cl.cloud, *cloud);
    if (cloud_is_good(cloud)) {
      return;
    }
    pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");

    pcl::PLYReader plyRead;
    plyRead.read(filename, *cloud);
    if (cloud_is_good(cloud)) {
      return;
    }

    pcl::console::print_error("\nError .ply file is not compatible.\n");
    std::exit(-1);
  }
};

class ParserTXT : public InterfaceParser {
 public:
  std::string parser_name = "ParserTXT";
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    std::ifstream file(filename, std::ifstream::in);
    if (!file.is_open()) {
      pcl::console::print_error("\nError: Could not find %s\n", filename);
      std::exit(-1);
    }

    double x_, y_, z_, r, g, b;

    while (file >> x_ >> y_ >> z_ >> r >> g >> b) {
      pcl::PointXYZRGB pt;
      pt.x = x_;
      pt.y = y_;
      pt.z = z_;

      uint8_t r_, g_, b_;
      r_ = uint8_t(r);
      g_ = uint8_t(g);
      b_ = uint8_t(b);

      uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_);
      pt.rgb = *reinterpret_cast<float*>(&rgb_);

      cloud->points.push_back(pt);
    }

    file.close();
    if (cloud_is_good(cloud)) {
      return;
    }

    pcl::console::print_error("\nError empty cloud.\n");
    std::exit(-1);
  }
};

class ParserXYZ : public InterfaceParser {
 public:
  std::string parser_name = "ParserXYZ";
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    std::ifstream file(filename, std::ifstream::in);
    if (!file.is_open()) {
      pcl::console::print_error("\nError: Could not find %s\n", filename);
      std::exit(-1);
    }

    double x_, y_, z_;

    while (file >> x_ >> y_ >> z_) {
      pcl::PointXYZRGB pt;
      pt.x = x_;
      pt.y = y_;
      pt.z = z_;

      cloud->points.push_back(pt);
    }

    file.close();
    if (cloud_is_good(cloud)) {
      return;
    }

    pcl::console::print_error("\nError empty cloud.\n");
    std::exit(-1);
  }
};

}  // namespace CloudParserLibrary
#endif