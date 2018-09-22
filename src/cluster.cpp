#include "cluster.h"

namespace dbScanSpace
{
	cluster::cluster()
	{
		visited = false;
	}

	void cluster::calculateCentroid()
	{
		if (clusterPoints.size() > 0)
		{
			for (auto& point : clusterPoints)
			{
				centroid.x += point.x;
				centroid.y += point.y;
				centroid.z += point.z;
			}
			centroid3D.x = centroid.x /= clusterPoints.size();
			centroid3D.y = centroid.y /= clusterPoints.size();
			centroid3D.z = centroid.z /= clusterPoints.size();
		}
	}

	///Converts the PCL points to htr points.
	void cluster::toPoint3D()
	{
		if (clusterPoints.size() > 0)
		{
			for (pcl::mod_pointXYZ point : clusterPoints)
			{
				htr::Point3D pointAux;
				pointAux.x = point.x;
				pointAux.y = point.y;
				pointAux.z = point.z;
				clusterPoints3D.push_back(pointAux);
			}
		}
	}
}