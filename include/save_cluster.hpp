#include <pcl/io/ply_io.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>

#include "dbscan/cluster.h"

std::string set_output_dir() {
  boost::filesystem::path dirPath(boost::filesystem::current_path());
  std::string output_dir = dirPath.string();
  output_dir += "/clusters";
  boost::filesystem::create_directory(output_dir);
  return output_dir;
}

void _to_pcd(std::vector<dbScanSpace::cluster>& clusters, std::string& output_dir) {
  int cont = 0;
  for (auto& cluster : clusters) {
    std::string str1 = output_dir;
    str1 += "/cluster_";
    str1 += std::to_string(cont);
    str1 += ".pcd";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (auto& point : cluster.clusterPoints) {
      pcl::PointXYZRGB pt;

      pt.x = point.x;
      pt.y = point.y;
      pt.z = point.z;

      pt.r = point.r;
      pt.g = point.g;
      pt.b = point.b;

      cloud_cluster_pcd->points.push_back(pt);
    }

    pcl::io::savePCDFileBinary(str1.c_str(), *cloud_cluster_pcd);
    cont += 1;
  }
}

void _to_ply(std::vector<dbScanSpace::cluster>& clusters, std::string& output_dir) {
  int cont = 0;
  for (auto& cluster : clusters) {
    std::string str1 = output_dir;
    str1 += "/cluster_";
    str1 += std::to_string(cont);
    str1 += ".ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_ply(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (auto& point : cluster.clusterPoints) {
      pcl::PointXYZRGB pt;

      pt.x = point.x;
      pt.y = point.y;
      pt.z = point.z;

      pt.r = point.r;
      pt.g = point.g;
      pt.b = point.b;

      cloud_cluster_ply->points.push_back(pt);
    }

    pcl::PLYWriter writer;
    writer.write(str1.c_str(), *cloud_cluster_ply, false, false);
    cont += 1;
  }
}

void _to_txt(std::vector<dbScanSpace::cluster>& clusters, std::string& output_dir) {
  int cont = 0;
  std::ofstream fout;
  for (auto& cluster : clusters) {
    std::string str1 = output_dir;
    str1 += "/cluster_";
    str1 += std::to_string(cont);
    str1 += ".txt";

    fout.open(str1.c_str());

    for (auto& point : cluster.clusterPoints) {
      uint32_t rgb_ = *reinterpret_cast<int*>(&point.rgb);
      uint8_t r_, g_, b_;

      r_ = (rgb_ >> 16) & 0x0000ff;
      g_ = (rgb_ >> 8) & 0x0000ff;
      b_ = (rgb_)&0x0000ff;

      unsigned int r, g, b;
      r = *((uint8_t*)&r_);
      g = *((uint8_t*)&g_);
      b = *((uint8_t*)&b_);

      fout << point.x << " " << point.y << " " << point.z << " " << r << " " << g << " " << b << std::endl;
    }

    fout.close();
    cont += 1;
  }
}

void _to_xyz(std::vector<dbScanSpace::cluster>& clusters, std::string& output_dir) {
  int cont = 0;
  std::ofstream fout;
  for (dbScanSpace::cluster& cluster_ : clusters) {
    std::string str1 = output_dir;
    str1 += "/cluster_";
    str1 += std::to_string(cont);
    str1 += ".xyz";

    fout.open(str1.c_str());

    for (auto& point : cluster_.clusterPoints) {
      fout << point.x << " " << point.y << " " << point.z << std::endl;
    }

    fout.close();
    cont += 1;
  }
}

void save_clusters(std::vector<dbScanSpace::cluster>& clusters, std::string& format, std::string& output_dir) {
  boost::filesystem::path dirPath(output_dir);

  if (not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)) {
    pcl::console::print_error("\nError. path %s does not exist or it's not valid", output_dir.c_str());
    std::exit(-1);
  }

  std::cout << "\n- output dir: " << output_dir << std::endl;
  const std::map<std::string, std::function<void()>> format_map{
      {"pcd", [&]() { _to_pcd(clusters, output_dir); }},
      {"ply", [&]() { _to_ply(clusters, output_dir); }},
      {"txt", [&]() { _to_txt(clusters, output_dir); }},
      {"xyz", [&]() { _to_xyz(clusters, output_dir); }},
  };

  static const auto end = format_map.end();
  auto it = format_map.find(format);
  if (it != end) {
    it->second();
    std::cout << "- cluster saved in format " << format << std::endl;
  } else {
    pcl::console::print_error("An exception occurred: Format %s is not supported\n");
    std::exit(-1);
  }
}