#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "MyMesh.h"
#include "pcl/common/distances.h"
#include "pcl/common/angles.h"
#include <pcl/filters/frustum_culling.h>
#include "VoxelGridOcclusionEstimationN.h"

typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
typedef std::pair<Eigen::Vector3f, Eigen::Vector3f>  Viewpoint;
using namespace std;

#define EPSILON 0.000001f

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
	float calculate_huristic(pcl::PointNormal currentPoint, pcl::PointNormal point, float cov, Eigen::Vector3f view_dir, pcl::PointNormal prevPoint);

	void  generate_path_layer(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, Viewpoints& viewpoints_list, pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud, float near, float far, float Hfov, float Vfov, float voxelRes, float curr_radius, SurfaceMesh& surface, Viewpoints& final_viewpoints, Tree& tree , vector<Eigen::Vector3f >& final_path,  Eigen::Vector3f* last_point = nullptr  );
	vector<Eigen::Vector3f > generate_path(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, Viewpoints& viewpoints_list, SurfaceMesh& surface, Viewpoints& final_viewpoints);

	vector<int> calculate_coverage(pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_viewpoints, pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud, float near, float far, float Hfov, float Vfov, float voxelRes);
}

