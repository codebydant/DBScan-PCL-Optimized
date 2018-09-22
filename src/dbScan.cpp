#include "dbScan.h"

namespace dbScanSpace
{
	dbscan::dbscan()
	{
		octreeGenIn = new htr::OctreeGenerator();
		octreeGen = new htr::OctreeGenerator();
	}

	dbscan::~dbscan()
	{
		delete octreeGenIn;
		delete octreeGen;
	}

	///Initializes the point cloud from a file, and the octree.
	///@param[in] filename          Location of the file that has the data.
	///@param[in] octreeResolution_ Resolution with which the octree is initialized.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] minPtsAux_        Minimum points for the initial clusters.
	///@param[in] minPts_           Minimum points for the final clusters.
	dbscan::dbscan(const char* filename, const int octreeResolution_, const float eps_, const int minPtsAux_, const int minPts_)
	{
		octreeGenIn = new htr::OctreeGenerator();
		octreeGen = new htr::OctreeGenerator();

		eps = eps_;
		minPts = minPts_;
		minPtsAux = minPtsAux_;
		octreeResolution = octreeResolution_;

		octreeGenIn->readCloudFromFile(filename);
		octreeGenIn->initOctree(octreeResolution);

		centroid = octreeGenIn->getCloudCentroid();
	}

	void dbscan::generateClusters_one_step()
	{
		clusters.clear();
		clustersAux.clear();
		clustersCentroids.clear();

		// A first set of clusters is generated. This first set has a large number of small clusters.
		DBSCAN_Octree_fast_one_step(octreeGenIn, eps, minPts);

		//// The clusters centroids are calculated and used to generate a second octree.
		//for (dbScanSpace::cluster cluster : clustersAux)
		//	clustersCentroids.push_back(cluster.centroid);

		//octreeGen->initCloudFromVector<pcl::mod_pointXYZ>(clustersCentroids);
		//octreeGen->initOctree(octreeResolution);

		//// Using the second octree and the centroids of the clusters, a new set of clusters is generated.
		//// These are the final clusters.
		//DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

		for (int i = 0; i<clusters.size(); i++)
			clusters[i].toPoint3D();
	}

	///Generates the clusters from the loaded data.
	void dbscan::generateClusters()
	{
		clusters.clear();
		clustersAux.clear();
		clustersCentroids.clear();

		// A first set of clusters is generated. This first set has a large number of small clusters.
		DBSCAN_Octree(octreeGenIn, eps, minPtsAux);

		// The clusters centroids are calculated and used to generate a second octree.
		for (dbScanSpace::cluster cluster : clustersAux)
			clustersCentroids.push_back(cluster.centroid);

		octreeGen->initCloudFromVector<pcl::mod_pointXYZ>(clustersCentroids);
		octreeGen->initOctree(octreeResolution);

		// Using the second octree and the centroids of the clusters, a new set of clusters is generated.
		// These are the final clusters.
		DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

		for (int i = 0; i<clusters.size(); i++)
			clusters[i].toPoint3D();
	}

	///Generates the clusters from the loaded data.
	void dbscan::generateClusters_fast()
	{
		clusters.clear();
		clustersAux.clear();
		clustersCentroids.clear();

		// A first set of clusters is generated. This first set has a large number of small clusters.
		//DBSCAN_Octree_fast2(octreeGenIn, minPtsAux);
		DBSCAN_Octree_fast(octreeGenIn, eps, minPtsAux);

		//        printf("\n Aux clusters size:%d\n\n", clustersAux.size());
		// The clusters centroids are calculated and used to generate a second octree.
		for (dbScanSpace::cluster cluster : clustersAux)
			clustersCentroids.push_back(cluster.centroid);

		octreeGen->initCloudFromVector<pcl::mod_pointXYZ>(clustersCentroids);
		octreeGen->initOctree(octreeResolution);

		// Using the second octree and the centroids of the clusters, a new set of clusters is generated.
		// These are the final clusters.
		//DBSCAN_Octree_merge(octreeGen, 2*eps, minPts);
		DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

		//        printf("\n Clusters size:%d\n\n", clusters.size());
		for (int i = 0; i<clusters.size(); i++)
			clusters[i].toPoint3D();
	}

	///Calculates the centroid form a vector of points.
	///@param[in] group             Vector that contains the point that will be processed.
	void dbscan::calculateCentroid(vector<pcl::mod_pointXYZ> group)
	{
		for (pcl::mod_pointXYZ point : group)
		{
			centroid.x += point.x;
			centroid.y += point.y;
			centroid.z += point.z;
		}
		centroid.x /= group.size();
		centroid.y /= group.size();
		centroid.z /= group.size();
	}

	///Does a radius search for the K nearest neighbors of a point.
	///@param[in] octreeGen         The octree to be searched.
	///@param[in] searchPoint       The point around which the search will be conducted.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] retKeys_          Vector that stores the indices of the nearest points.
	void dbscan::octreeRegionQuery(htr::OctreeGenerator *octreeGen, pcl::mod_pointXYZ & searchPoint, double eps, vector<int> *retKeys)
	{
		retKeys->clear();
		vector<int> pointIdxRadiusSearch;
		vector<float> pointRadiusSquaredDistance;

		octreeGen->getOctree()->radiusSearch(searchPoint, eps, *retKeys, pointRadiusSquaredDistance);
	}

	///Merges a set of clusters.
	///@param[in] octreeGen         The octree to be searched.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] clustersIn        The clusters that will be merged.
	///@param[in] clustersOut       Vector that stores all merged clusters.
	void dbscan::DBSCAN_Octree_merge(htr::OctreeGenerator *octreeGen, float eps, int minPts)
	{
		clusters.clear();
		cluster pointQueue;
		vector<pcl::mod_pointXYZ> clusterPoints;

		// The amount of aux clusters
		int noKeys = clustersAux.size();
		vector<bool> visited(noKeys, false);

		vector<int> noise;
		vector<int> neighborPts(noKeys, -1);

		for (int i = 0; i < noKeys; i++)
		{
			if (!visited[i])
			{
				clusterPoints.push_back(clustersAux.at(i).centroid);

				pointQueue.clusterPoints.insert(pointQueue.clusterPoints.end(),
					clustersAux.at(i).clusterPoints.begin(),
					clustersAux.at(i).clusterPoints.end());

				visited[i] = true;

				for (int j = 0; j < clusterPoints.size(); j++)
				{
					octreeRegionQuery(octreeGen, clusterPoints.at(j), eps, &neighborPts);

					for (int k = 0; k < neighborPts.size(); k++)
					{
						if (!visited[neighborPts[k]])
						{
							visited[neighborPts[k]] = true;

							clusterPoints.push_back(clustersAux.at(neighborPts[k]).centroid);

							pointQueue.clusterPoints.insert(pointQueue.clusterPoints.end(),
								clustersAux.at(neighborPts[k]).clusterPoints.begin(),
								clustersAux.at(neighborPts[k]).clusterPoints.end());
						}
					}
				}

				if (pointQueue.clusterPoints.size() >= minPts)
				{
					pointQueue.calculateCentroid();
					clusters.push_back(pointQueue);
					pointQueue.clusterPoints.clear();
				}
			}
		}

		//       clustersAux.clear();
	}

	///Generates a set of clusters.
	///@param[in] octreeGen         The octree to be searched.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] clusters          Vector that stores all the generated clusters.
	void dbscan::DBSCAN_Octree(htr::OctreeGenerator *octreeGen, float eps, int minPts)
	{
		clustersAux.clear();
		cluster pointQueue;
		pcl::mod_pointXYZ auxCentroid;

		int noKeys = octreeGen->getCloud()->points.size();
		vector<bool> visited(noKeys, false);
		vector<int> classification(noKeys, 0);

		vector<int> noise;
		vector<int> neighborPts;

		for (int i = 0; i < noKeys; i++)
		{
			if (!visited[i])
			{
				pointQueue.clusterPoints.push_back(octreeGen->getCloud()->points.at(i));
				visited[i] = true;

				octreeRegionQuery(octreeGen, pointQueue.clusterPoints.at(0), eps, &neighborPts);

				if (neighborPts.size() < minPtsAux)
					noise.push_back(i);
				else
				{
					for (int k = 0; k < neighborPts.size(); k++)
					{
						if (!visited[neighborPts[k]])
						{
							visited[neighborPts[k]] = true;
							pcl::mod_pointXYZ auxPoint = octreeGen->getCloud()->points.at(neighborPts[k]);
							pointQueue.clusterPoints.push_back(auxPoint);
						}
					}

					if (pointQueue.clusterPoints.size() >= minPtsAux)
					{
						pointQueue.calculateCentroid();
						clustersAux.push_back(pointQueue);
						pointQueue.clusterPoints.clear();
					}
				}
			}
		}
	}

	///Generates a set of clusters.
	///@param[in] octreeGen         The octree to be searched.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] clusters          Vector that stores all the generated clusters.
	void dbscan::DBSCAN_Octree_fast(htr::OctreeGenerator *octreeGen, float eps, int minPts) // O(nlogn)
	{
		clustersAux.clear();

		cluster pointQueue;
		pcl::mod_pointXYZ auxCentroid;

		// The number of points
		int noKeys = octreeGen->getCloud()->points.size();
		vector<bool> visited(noKeys, false);
		vector<int> classification(noKeys, 0);

		vector<int> noise;
		vector<int> neighborPts;

		for (int i = 0; i < noKeys; i++)	// O(n)
		{
			if (!visited[i]) // O(log n)
			{
				pointQueue.clusterPoints.push_back(octreeGen->getCloud()->points.at(i));
				visited[i] = true;

				octreeGen->getOctree()->voxelSearch(pointQueue.clusterPoints.at(0), neighborPts); 

				if (neighborPts.size() < minPtsAux)
					noise.push_back(i);
				else
				{
					for (int k = 0; k < neighborPts.size(); k++)
					{
						if (!visited[neighborPts[k]])
						{
							visited[neighborPts[k]] = true;
							pcl::mod_pointXYZ auxPoint = octreeGen->getCloud()->points.at(neighborPts[k]);
							pointQueue.clusterPoints.push_back(auxPoint);
						}
					}

					if (pointQueue.clusterPoints.size() >= minPtsAux)
					{
						pointQueue.calculateCentroid();
						clustersAux.push_back(pointQueue);
						pointQueue.clusterPoints.clear();
					}
				}
			}
		}
	}

	///Generates a set of clusters.
	///@param[in] octreeGen         The octree to be searched.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] clusters          Vector that stores all the generated clusters.
	void dbscan::DBSCAN_Octree_fast_one_step(htr::OctreeGenerator *octreeGen, float eps, int minPts)
	{
		int noKeys = octreeGen->getCloud()->points.size();
		vector<bool> clustered(noKeys, false);
		vector<bool> visited(noKeys, false);

		vector<int> noise;

		vector<int> neighborPts;
		vector<int> neighborPts_;

		int c = 0;

		//for each unvisted point P in dataset keypoints
		for (int i = 0; i < noKeys; i++)
		{
			if (!visited[i])
			{
				//Mark P as visited
				visited[i] = true;
				octreeRegionQuery(octreeGen, octreeGen->getCloud()->points.at(i), eps, &neighborPts);

				if (neighborPts.size() < minPts)
					//Mark P as Noise
					noise.push_back(i);
				else
				{
					clusters.push_back(cluster());

					//expand cluster, add P to cluster c
					clustered[i] = true;
					clusters.at(c).clusterPoints.push_back(octreeGen->getCloud()->points.at(i));

					//for each point P' in neighborPts, Expand cluster
					for (int j = 0; j < neighborPts.size(); j++)
					{
						//                        if P' is not visited
						if (!visited[neighborPts[j]])
						{
							//Mark P' as visited
							visited[neighborPts[j]] = true;
							octreeRegionQuery(octreeGen, octreeGen->getCloud()->points.at(neighborPts[j]),
								eps, &neighborPts_);
							//
							if (neighborPts_.size() >= minPts)
								neighborPts.insert(neighborPts.end(), neighborPts_.begin(), neighborPts_.end());

						}
						// if P' is not yet a member of any cluster, add P' to cluster c
						if (!clustered[neighborPts[j]])
						{
							clustered[neighborPts[j]] = true;
							clusters.at(c).clusterPoints.push_back(octreeGen->getCloud()->points.at(neighborPts[j]));
						}
					}
					c++;
				}

			}
		}
	}

	///Generates a set of clusters.
	///@param[in] octreeGen         The octree to be searched.
	///@param[in] eps_              The search radius for the octree.
	///@param[in] clusters          Vector that stores all the generated clusters.
	void dbscan::DBSCAN_Octree_fast2(htr::OctreeGenerator *octreeGen, int minPts)
	{
		//Clear aux clusters
		clustersAux.clear();

		if (!octreeGen->getCloud()->points.empty())
		{
			//Create array of tree iterators
			vector<htr::OctreeGenerator::LeafNodeIterator> leafIterators;

			vector<int> tempVec;
			cluster tempPointCluster;
			vector<vector<int>> neighborPts;

			htr::OctreeGenerator::LeafNodeIterator it(octreeGen->getOctree().get());

			while (*(it))
			{
				neighborPts.push_back(tempVec);
				clustersAux.push_back(tempPointCluster);

				it.getLeafContainer().getPointIndices(neighborPts.back());
				for (int k = 0; k<neighborPts.back().size(); ++k)
				{
					clustersAux.back().clusterPoints.push_back(octreeGen->getCloud()->points.at(neighborPts.back().at(k)));
				}
				clustersAux.back().calculateCentroid();

				++it;
			}
		}

	}
}

//#include "dbScan.h"
//
//namespace dbScanSpace
//{
//
//dbscan::dbscan()
//{
//    octreeGenIn = new htr::OctreeGenerator();
//    octreeGen = new htr::OctreeGenerator();
//}
//
//dbscan::~dbscan()
//{
//    delete octreeGenIn;
//    delete octreeGen;
//}
//
//   ///Initializes the point cloud from a file, and the octree.
/////@param[in] filename          Location of the file that has the data.
/////@param[in] octreeResolution_ Resolution with which the octree is initialized.
/////@param[in] eps_              The search radius for the octree.
/////@param[in] minPtsAux_        Minimum points for the initial clusters.
/////@param[in] minPts_           Minimum points for the final clusters.
//   dbscan::dbscan(const char* filename, const int octreeResolution_, const float eps_, const int minPtsAux_, const int minPts_ )
//   {
//       octreeGenIn = new htr::OctreeGenerator();
//       octreeGen = new htr::OctreeGenerator();
//
//       eps = eps_;
//       minPts = minPts_;
//       minPtsAux = minPtsAux_;
//       octreeResolution = octreeResolution_;
//
//       octreeGenIn->readCloudFromFile(filename);
//       octreeGenIn->initOctree(octreeResolution);
//
//       centroid = octreeGenIn->getCloudCentroid();
//   }
//
//   ///Generates the clusters from the loaded data.
//   void dbscan::generateClusters()
//   {
//       clusters.clear();
//       clustersAux.clear();
//       clustersCentroids.clear();
//
//       // A first set of clusters is generated. This first set has a large number of small clusters.
//       DBSCAN_Octree(octreeGenIn, eps, minPtsAux);
//
//       // The clusters centroids are calculated and used to generate a second octree.
//       for(dbScanSpace::cluster cluster:clustersAux)
//           clustersCentroids.push_back(cluster.centroid);
//
//       octreeGen->initCloudFromVector<pcl::PointXYZ>(clustersCentroids);
//       octreeGen->initOctree(octreeResolution);
//
//       // Using the second octree and the centroids of the clusters, a new set of clusters is generated.
//       // These are the final clusters.
//       DBSCAN_Octree_merge(octreeGen, 2*eps, minPts);
//
//       for(int i = 0; i<clusters.size(); i++)
//           clusters[i].toPoint3D();
//   }
//
//   ///Generates the clusters from the loaded data.
//   void dbscan::generateClusters_fast()
//   {
//       clusters.clear();
//       clustersAux.clear();
//       clustersCentroids.clear();
//
//       // A first set of clusters is generated. This first set has a large number of small clusters.
//       DBSCAN_Octree_fast2(octreeGenIn, minPtsAux);
//
////        printf("\n Aux clusters size:%d\n\n", clustersAux.size());
//       // The clusters centroids are calculated and used to generate a second octree.
//       for(dbScanSpace::cluster cluster:clustersAux)
//           clustersCentroids.push_back(cluster.centroid);
//
//       octreeGen->initCloudFromVector<pcl::PointXYZ>(clustersCentroids);
//       octreeGen->initOctree(octreeResolution);
//
//       // Using the second octree and the centroids of the clusters, a new set of clusters is generated.
//       // These are the final clusters.
//       //DBSCAN_Octree_merge(octreeGen, 2*eps, minPts);
//	   DBSCAN_Octree_merge(octreeGen, 2*eps, minPts);
//
////        printf("\n Clusters size:%d\n\n", clusters.size());
//       for(int i = 0; i<clusters.size(); i++)
//           clusters[i].toPoint3D();
//   }
//
//   ///Calculates the centroid form a vector of points.
/////@param[in] group             Vector that contains the point that will be processed.
//   void dbscan::calculateCentroid(vector<pcl::PointXYZ> group)
//   {
//       for(pcl::PointXYZ point:group)
//       {
//           centroid.x+=point.x;
//           centroid.y+=point.y;
//           centroid.z+=point.z;
//       }
//       centroid.x/=group.size();
//       centroid.y/=group.size();
//       centroid.z/=group.size();
//   }
//
//   ///Does a radius search for the K nearest neighbors of a point.
/////@param[in] octreeGen         The octree to be searched.
/////@param[in] searchPoint       The point around which the search will be conducted.
/////@param[in] eps_              The search radius for the octree.
/////@param[in] retKeys_          Vector that stores the indices of the nearest points.
//   void dbscan::octreeRegionQuery(htr::OctreeGenerator *octreeGen, pcl::PointXYZ & searchPoint, double eps, vector<int> *retKeys)
//   {
//       retKeys->clear();
//       vector<int> pointIdxRadiusSearch;
//       vector<float> pointRadiusSquaredDistance;
//
//       octreeGen->getOctree()->radiusSearch(searchPoint, eps, *retKeys, pointRadiusSquaredDistance);
//   }
//
//   ///Merges a set of clusters.
/////@param[in] octreeGen         The octree to be searched.
/////@param[in] eps_              The search radius for the octree.
/////@param[in] clustersIn        The clusters that will be merged.
/////@param[in] clustersOut       Vector that stores all merged clusters.
//   void dbscan::DBSCAN_Octree_merge(htr::OctreeGenerator *octreeGen, float eps, int minPts)
//   {
//       clusters.clear();
//       cluster pointQueue;
//       vector<pcl::PointXYZ> clusterPoints;
//
//       int noKeys = clustersAux.size();
//       vector<bool> visited(noKeys, false);
//
//       vector<int> noise;
//       vector<int> neighborPts(noKeys, -1);
//
//       for(int i = 0; i < noKeys; i++)
//       {
//           if(!visited[i])
//           {
//               clusterPoints.push_back(clustersAux.at(i).centroid);
//               pointQueue.clusterPoints.insert(pointQueue.clusterPoints.end(),
//                                               clustersAux.at(i).clusterPoints.begin(),
//                                               clustersAux.at(i).clusterPoints.end());
//               visited[i] = true;
//
//               for(int j = 0; j < clusterPoints.size(); j++)
//               {
//                   octreeRegionQuery(octreeGen, clusterPoints.at(j), eps, &neighborPts);
//
//                   for(int k =0; k < neighborPts.size(); k++)
//                   {
//                       if(!visited[neighborPts[k]])
//                       {
//                           visited[neighborPts[k]] = true;
//
//                           clusterPoints.push_back(clustersAux.at(neighborPts[k]).centroid);
//
//                           pointQueue.clusterPoints.insert(pointQueue.clusterPoints.end(),
//                                                           clustersAux.at(neighborPts[k]).clusterPoints.begin(),
//                                                           clustersAux.at(neighborPts[k]).clusterPoints.end());
//                       }
//                   }
//               }
//
//               if(pointQueue.clusterPoints.size() >= minPts)
//               {
//                   pointQueue.calculateCentroid();
//                   clusters.push_back(pointQueue);
//               }
//
//               pointQueue.clusterPoints.clear();
//           }
//       }
//
////       clustersAux.clear();
//   }
//
//   ///Generates a set of clusters.
/////@param[in] octreeGen         The octree to be searched.
/////@param[in] eps_              The search radius for the octree.
/////@param[in] clusters          Vector that stores all the generated clusters.
//   void dbscan::DBSCAN_Octree(htr::OctreeGenerator *octreeGen, float eps, int minPts)
//   {
//       clustersAux.clear();
//       cluster pointQueue;
//       pcl::PointXYZ auxCentroid;
//
//       int noKeys = octreeGen->getCloud()->points.size();
//       vector<bool> visited(noKeys, false);
//       vector<int> classification(noKeys, 0);
//
//       vector<int> noise;
//       vector<int> neighborPts;
//
//       for(int i = 0; i < noKeys; i++)
//       {
//           if(!visited[i])
//           {
//               pointQueue.clusterPoints.push_back(octreeGen->getCloud()->points.at(i));
//               visited[i] = true;
//
//				octreeRegionQuery(octreeGen, pointQueue.clusterPoints.at(0),eps, &neighborPts);
//
//                   for(int k =0; k < neighborPts.size(); k++)
//                   {
//                       if(!visited[neighborPts[k]])
//                       {
//                           visited[neighborPts[k]] = true;
//                           pcl::PointXYZ auxPoint = octreeGen->getCloud()->points.at(neighborPts[k]);
//                           pointQueue.clusterPoints.push_back(auxPoint);
//                       }
//                   }
//
//               if(pointQueue.clusterPoints.size() >= minPtsAux)
//               {
//                   pointQueue.calculateCentroid();
//                   clustersAux.push_back(pointQueue);
//               }
//
//               pointQueue.clusterPoints.clear();
//           }
//       }
//   }
//
//  ///Generates a set of clusters.
/////@param[in] octreeGen         The octree to be searched.
/////@param[in] eps_              The search radius for the octree.
/////@param[in] clusters          Vector that stores all the generated clusters.
//   void dbscan::DBSCAN_Octree_fast(htr::OctreeGenerator *octreeGen, float eps, int minPts)
//   {
//       clustersAux.clear();
//
//       cluster pointQueue;
//       pcl::PointXYZ auxCentroid;
//
//       int noKeys = octreeGen->getCloud()->points.size();
//       vector<bool> visited(noKeys, false);
//       vector<int> classification(noKeys, 0);
//
//       vector<int> noise;
//       vector<int> neighborPts;
//
//       for(int i = 0; i < noKeys; i++)
//       {
//           if(!visited[i])
//           {
//               pointQueue.clusterPoints.push_back(octreeGen->getCloud()->points.at(i));
//               visited[i] = true;
//
//                octreeGen->getOctree()->voxelSearch(pointQueue.clusterPoints.at(0), neighborPts);
//
//                   for(int k =0; k < neighborPts.size(); k++)
//                   {
//                       if(!visited[neighborPts[k]])
//                       {
//                           visited[neighborPts[k]] = true;
//                           pcl::PointXYZ auxPoint = octreeGen->getCloud()->points.at(neighborPts[k]);
//                           pointQueue.clusterPoints.push_back(auxPoint);
//                       }
//                   }
//
//               if(pointQueue.clusterPoints.size() >= minPtsAux)
//               {
//                   pointQueue.calculateCentroid();
//                   clustersAux.push_back(pointQueue);
//               }
//
//               pointQueue.clusterPoints.clear();
//           }
//       }
//   }
//
//
//	///Generates a set of clusters.
//	///@param[in] octreeGen         The octree to be searched.
//	///@param[in] eps_              The search radius for the octree.
//	///@param[in] clusters          Vector that stores all the generated clusters.
//	void dbscan::DBSCAN_Octree_fast2(htr::OctreeGenerator *octreeGen, int minPts)
//	{
//		//Clear aux clusters
//		clustersAux.clear();
//
//		if(!octreeGen->getCloud()->points.empty())
//		{
//			//Create array of tree iterators
//			vector<htr::OctreeGenerator::LeafNodeIterator> leafIterators;
//
//		    vector<int> tempVec;
//		    cluster tempPointCluster;
//			vector<vector<int>> neighborPts;
//
//			htr::OctreeGenerator::LeafNodeIterator it(octreeGen->getOctree().get());
//
//			while ( *(it) )
//			{
//				neighborPts.push_back(tempVec);
//				clustersAux.push_back(tempPointCluster);
//
//				it.getLeafContainer().getPointIndices(neighborPts.back());
//				for(int k=0; k<neighborPts.back().size(); ++k)
//				{
//					clustersAux.back().clusterPoints.push_back(octreeGen->getCloud()->points.at(neighborPts.back().at(k)));
//				}
//				clustersAux.back().calculateCentroid();
//
//				++it;
//			}
//		}
//
//	}
//}
