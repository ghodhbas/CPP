#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "MyMesh.h"

typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
typedef std::pair<Eigen::Vector3f, Eigen::Vector3f>  Viewpoint;
using namespace std;

class Viewpoint_Comparator {
public:
	bool operator() (const Viewpoint& v1, const Viewpoint& v2) {
		return v1.first[1] < v2.first[1];
	}

	bool operator() (const pcl::PointNormal& v1, const pcl::PointNormal& v2) {
		return v1.y < v2.y;
	}
};


namespace ExploratoryPlanner
{	
	float calculate_huristic(pcl::PointNormal currentPoint, pcl::PointNormal point);

	void generate_path(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, SurfaceMesh& surface);
}

