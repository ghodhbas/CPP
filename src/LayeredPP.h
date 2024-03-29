#pragma once
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <vector>

#include "MyMesh.h"
#include "graph.h"

class LayeredPP
{	
public:
	LayeredPP( float near, float far);

	vector< pcl::PointCloud<pcl::PointNormal>::Ptr> construct_layers(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const int nb_layers, const std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max);
	vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> voxelize_layers(vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layers, float voxelRes);
	pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> voxelize(pcl::PointCloud<pcl::PointNormal>::Ptr&  cloud, float voxelRes);


	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> generate_viewpoints(pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> layer, Polyhedron& poly);
	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> generate_seg_viewpoints(pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> layer, Polyhedron& seg_poly, Polyhedron& poly);
	

	std::map<int, std::map<int, float>>  calculate_distances(std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>&  viewpoints, vector<std::pair<int, int>>& pair_vec, SurfaceMesh& surface, std::pair<pcl::PointXYZ, pcl::PointXYZ>& bbox, Tree& tree);

	Edge_Graph construct_MST(vector<std::pair<int, int>>& pair_vec , std::map<int, std::map<int, float>>& distance_map);
	vector<Eigen::Vector3f> generate_path(Edge_Graph& MST, std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& layer, Eigen::Vector3f& last_node);

	void output_viewpoints(std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>>& layer_viewpoints);

	float get_near_plane_d() { return near_plane_d; }
	float get_far_plane_d() { return far_plane_d; }

private:
	int get_range(pcl::PointNormal point, vector < std::pair<float, float>>& ranges);

	vector<std::pair<float, float>> construct_cutoff_ranges(const int nb_layers, const std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max);
	void classify_points_to_layers(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, vector<std::pair<float, float>>& ranges, vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layer_vec);


	void combinations_recursive(const vector<int>& elems, unsigned long req_len,
		vector<unsigned long>& pos, unsigned long depth,
		unsigned long margin, vector<std::pair<int, int>>& pair_vec);
	void combinations(const vector<int>& elems, vector<std::pair<int, int>>& pair_vec);
	void DFS(vector<Eigen::Vector3f>& path, Node* node, Node* prev);

	float near_plane_d;
	float far_plane_d;
	float distance_epsilon = 0.05f;
};

