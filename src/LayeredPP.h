#pragma once
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <vector>

#include "MyMesh.h"

class LayeredPP
{	
public:
	LayeredPP(float resolution);

	vector< pcl::PointCloud<pcl::PointNormal>::Ptr> construct_layers(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const int nb_layers, const std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max);
	vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> voxelize_layers(vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layers);


private:
	int get_range(pcl::PointNormal point, vector < std::pair<float, float>>& ranges);

	vector<std::pair<float, float>> construct_cutoff_ranges(const int nb_layers, const std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max);
	void classify_points_to_layers(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, vector<std::pair<float, float>>& ranges, vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layer_vec);



	float voxelRes;
};

