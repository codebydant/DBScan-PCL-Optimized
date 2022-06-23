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
#include "clusters_color.hpp"
#include "dbscan/dbScan.h"
#include "modern/parser.hpp"
#include "save_cluster.hpp"

int main(int argc, char **argv) {
  std::cout << "\n*************************************" << std::endl;
  std::cout << "*** DBSCAN Cluster Segmentation *** " << std::endl;
  std::cout << "*************************************" << std::endl;

  argparse::ArgumentParser arg_parser(argv[0]);

  arg_parser.add_argument("--cloudfile").required().help("input cloud file");
  arg_parser.add_argument("--octree-res").default_value(int(120)).scan<'i', int>().help("octree resolution");
  arg_parser.add_argument("--eps").default_value(float(40)).scan<'g', float>().help("epsilon value");
  arg_parser.add_argument("--minPtsAux").default_value(int(5)).scan<'i', int>().help("minimum auxiliar points");
  arg_parser.add_argument("--minPts").default_value(int(5)).scan<'i', int>().help("minimum points");

  arg_parser.add_argument("-o", "--output-dir")
      .default_value(std::string("-"))
      .help("output dir to save clusters");

  arg_parser.add_argument("--ext")
      .default_value(std::string("pcd"))
      .help("cluster output extension [pcd, ply, txt, xyz]");

  arg_parser.add_argument("-d", "--display")
      .default_value(false)
      .implicit_value(true)
      .help("display clusters in the pcl visualizer");

  try {
    arg_parser.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << arg_parser;
    std::exit(1);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  // cloud parser object
  CloudParserLibrary::ParserCloudFile cloud_parser;
  cloud_parser.load_cloudfile(arg_parser.get<std::string>("--cloudfile"), cloud);

  // set cloud metadata
  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  /*RUN DBSCAN

   DBSCAN algorithm requires 3 parameters:

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

  // -----------------Visualize clusters -----------------
  if (arg_parser["--display"] == true) {
    // Disable vtk render warning
    vtkObject::GlobalWarningDisplayOff();

    // pcl visualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL VISUALIZER"));

    int PORT1 = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
    viewer->setBackgroundColor(0, 0, 0, PORT1);
    viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

    int PORT2 = 0;
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
    viewer->setBackgroundColor(0, 0, 0, PORT2);
    viewer->addText("CLUSTERS", 10, 10, "PORT2", PORT2);

    viewer->setPosition(0, 0);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0);  // Setting background to a dark

    int numClust = 0;
    std::random_device seeder;
    std::ranlux48 gen(seeder());
    std::uniform_int_distribution<int> uniform_0_255(0, 255);

    int j = 0;
    for (auto &cluster : dbscan.getClusters()) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

      uint8_t r;
      uint8_t g;
      uint8_t b;

      if (j < 60) {
        r = (uint8_t)colors[j];
        g = (uint8_t)colors[j + 1];
        b = (uint8_t)colors[j + 2];

      } else {
        r = (uint8_t)uniform_0_255(gen);
        g = (uint8_t)uniform_0_255(gen);
        b = (uint8_t)uniform_0_255(gen);
      }

      // Adding different color to each cluster
      for (auto &pointCluster : cluster.clusterPoints) {
        pcl::PointXYZRGB point;
        point.x = pointCluster.x;
        point.y = pointCluster.y;
        point.z = pointCluster.z;

        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);

        cluster_rgb->points.push_back(point);
      }

      j += 3;

      std::string nameId = "cluster_";
      nameId += std::to_string(numClust);

      viewer->addPointCloud(cluster_rgb, nameId.c_str(), PORT2);
      numClust += 1;
    }

    double scale = 1;

    viewer->addCoordinateSystem(scale);
    pcl::PointXYZ p11, p22, p33;
    p11.getArray3fMap() << 1, 0, 0;
    p22.getArray3fMap() << 0, 1, 0;
    p33.getArray3fMap() << 0, 0.1, 1;

    viewer->addText3D("x", p11, 0.2, 1, 0, 0, "x_");
    viewer->addText3D("y", p22, 0.2, 0, 1, 0, "y_");
    viewer->addText3D("z", p33, 0.2, 0, 0, 1, "z_");

    if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b <= 0) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
      viewer->addPointCloud(cloud, color_handler, "Original_Cloud", PORT1);
    } else {
      viewer->addPointCloud(cloud, "Original_Cloud", PORT1);
    }

    viewer->initCameraParameters();
    viewer->resetCamera();

    pcl::console::print_info("\npress [q] to exit!\n");

    while (!viewer->wasStopped()) {
      viewer->spin();
    }
  }

  // true, if user provided output_dir
  if (arg_parser.is_used("--output-dir")) {
    std::string output_dir = arg_parser.get<std::string>("--output-dir");
    std::string format = arg_parser.get<std::string>("--ext");
    save_clusters(dbscan.getClusters(), format, output_dir);
  }

  return 0;
}
