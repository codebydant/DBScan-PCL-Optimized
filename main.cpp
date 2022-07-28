/*
DBSCAN, or Density-Based Spatial Clustering of Applications with Noise,
is an unsupervised machine learning algorithm. Unsupervised machine learning
algorithms are used to classify unlabeled data.

DBSCAN is particularly well suited for problems which require:
1. Minimal domain knowledge to determine the input parameters (i.e. K in k-means and Dmin in hierarchical
clustering)
2. Discovery of clusters with arbitrary shapes
3. Good efficiency on large databases

As is the case in most machine learning algorithms, the model’s behaviour is dictated by several parameters.

1. eps: Two points are considered neighbors if the distance between the two points is below the threshold epsilon.
2. min_samples: The minimum number of neighbors a given point should have in order to be classified as a core
point. It’s important to note that the point itself is included in the minimum number of samples.

The algorithm works by computing the distance between every point and all other points. We then place the points
into one of three categories.
1. Core point: A point with at least min_samples points whose distance with respect to the point is below the
threshold defined by epsilon.
2. Border point: A point that isn’t in close proximity to at least min_samples points but is close enough to one or
more core point. Border points are included in the cluster of the closest core point.
3. Noise point: Points that aren’t close enough to core points to be considered border points. Noise points are
ignored. That is to say, they aren’t part of any cluster.
*/

#include <iostream>

#include "argparse/argparse.hpp"
#include "cal_epsilon.hpp"
#include "cloudparse/parser.hpp"
#include "clusters_color.hpp"
#include "dbscan/dbScan.h"
#include "save_cluster.hpp"
#include "visualizer.hpp"

int main(int argc, char **argv) {
  std::cout << "\n*************************************" << std::endl;
  std::cout << "*** DBSCAN Cluster Segmentation *** " << std::endl;
  std::cout << "*************************************" << std::endl;

  // -----------------Command line interface -----------------
  argparse::ArgumentParser arg_parser(argv[0]);

  arg_parser.add_argument("--cloudfile").required().help("input cloud file");
  arg_parser.add_argument("--octree-res").default_value(int(120)).scan<'i', int>().help("octree resolution");
  arg_parser.add_argument("--eps").default_value(float(40)).scan<'g', float>().help("epsilon value");
  arg_parser.add_argument("--minPtsAux").default_value(int(5)).scan<'i', int>().help("minimum auxiliar points");
  arg_parser.add_argument("--minPts").default_value(int(5)).scan<'i', int>().help("minimum points");
  arg_parser.add_argument("-o", "--output-dir").default_value(std::string("-")).help("output dir to save clusters");
  arg_parser.add_argument("--ext").default_value(std::string("pcd")).help("cluster output extension [pcd, ply, txt, xyz]");
  arg_parser.add_argument("-d", "--display").default_value(false).implicit_value(true).help("display clusters in the pcl visualizer");
  arg_parser.add_argument("--cal-eps").default_value(false).implicit_value(true).help("calculate the value of epsilon with the distance to the nearest n points");

  try {
    arg_parser.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << arg_parser;
    std::exit(0);
  }

  // -----------------Read input cloud file -----------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  // cloud parser object
  CloudParserLibrary::ParserCloudFile cloud_parser;
  cloud_parser.load_cloudfile(arg_parser.get<std::string>("--cloudfile"), cloud);

  // set cloud metadata
  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  // -----------------RUN DBSCAN -----------------
  /*DBSCAN algorithm requires 3 parameters:

   - octreeResolution: describes the length of the smallest voxels at lowest octree level.
   - epsilon: specifies how close points should be to each other to be considered a part of a cluster
   - minPts: specifies how many neighbors a point should have to be included into a cluster (must be >=3).

   Note:
   epsilone can be set as groupA.size()*0.001

   e.g.
   dbscan.init(groupA, groupA.size()*0.001, groupA.size()*0.001, 10, 100);
  */

  std::vector<htr::Point3D> groupA;
  dbScanSpace::dbscan dbscan;

  int octreeResolution = arg_parser.get<int>("--octree-res");
  float eps = arg_parser.get<float>("--eps");
  int minPtsAux = arg_parser.get<int>("--minPtsAux");
  int minPts = arg_parser.get<int>("--minPts");

  if (minPts < 3) {
    pcl::console::print_error("\nminPts must be >= 3! \n");
    std::exit(-1);
  }

  pcl::console::print_info("\n- octreeResolution: %i", octreeResolution);
  pcl::console::print_info("\n- eps: %f ", eps);
  pcl::console::print_info("\n- minPtsAux: %i", minPtsAux);
  pcl::console::print_info("\n- minPts: %i\n", minPts);

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  dbscan.init(groupA, cloud, octreeResolution, eps, minPtsAux, minPts);
  // dbscan.init(groupA, cloud, cloud->points.size() * 0.001, eps, 5, 100);
  dbscan.generateClusters();
  // dbscan.generateClusters_fast();
  // dbscan.generateClusters_one_step();

  end = std::chrono::system_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  pcl::console::print_info("\n- elapsed time: ");
  pcl::console::print_value("%d ms", elapsed_ms);

  if (dbscan.getClusters().size() <= 0) {
    pcl::console::print_error("\nCould not generated clusters, bad parameters\n");
    std::exit(-1);
  }

  pcl::console::print_info("\n- clusters: ");
  pcl::console::print_value("%d\n", dbscan.getClusters().size());

  // -----------------Calculate epsilon -----------------
  if (arg_parser["--cal-eps"] == true) {
    std::cout << "- calculating epsilon..." << std::endl;
    calculate_epsilon(cloud);
  }

  // -----------------Visualize clusters -----------------
  if (arg_parser["--display"] == true) {
    display_clusters(cloud, dbscan.getClusters());
  }

  // -----------------Export clusters -----------------
  // true, if user provided output_dir
  if (arg_parser.is_used("--output-dir")) {
    std::string output_dir = arg_parser.get<std::string>("--output-dir");
    std::string format = arg_parser.get<std::string>("--ext");
    save_clusters(dbscan.getClusters(), format, output_dir);
  }

  return 0;
}
