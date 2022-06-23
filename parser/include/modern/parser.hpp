/*
 * Implementation of a parser factory for cloud parses
 *
 * it implements a factory design pattern to create a parser from a concrete class.
 * each concrete parser is derived from an abstract class InterfaceParser and this
 * module provides a ParserFactory class to register new formats.
 *
 * ParserCloudFile is the common interface to load a cloud file given a path. This
 * interface will check if the current file extension is supported by the factories list.
 * In case there is a provided format, a parser object extension will be provided to the client.
 *
 * Current supported formats are: [.pcd, .ply]
 */

#pragma once
#ifndef PARSER_HPP
#define PARSER_HPP
#include "concrete_parses.hpp"

namespace CloudParserLibrary {

class ParserFactory {
 public:
  void register_format(std::string format, InterfaceParser *ptr);
  InterfaceParser *get_parser(const std::string format);
  size_t get_size();

 private:
  std::map<std::string, InterfaceParser *> factories = {};
};

class ParserCloudFile {
 public:
  ParserFactory parser_factory;
  ParserCloudFile() {
    parser_factory.register_format("PCD", new ParserPCD());
    parser_factory.register_format("PLY", new ParserPLY());
    parser_factory.register_format("TXT", new ParserTXT());
    parser_factory.register_format("XYZ", new ParserXYZ());
  }
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    int position = filename.find_last_of(".");
    std::string extension = filename.substr(position + 1);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::toupper);
    InterfaceParser *cloudparser = parser_factory.get_parser(extension);
    pcl::console::TicToc tt;
    pcl::console::print_highlight("Loading ");
    cloudparser->load_cloudfile(filename, cloud);
    pcl::console::print_info("\nFound %s file.\n", extension.c_str());
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->size());
    pcl::console::print_info(" points]\n");
  }
};

}  // namespace CloudParserLibrary
#endif