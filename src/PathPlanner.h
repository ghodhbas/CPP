#pragma once
#include "Occlusion_Culling.h"
#include <vector>

#include "MyMesh.h"
#include "graph.hpp"
#define EPSILON 0.000001f

class PathPlanner
{
    // at this point the frustum culling wroks on verticies/point cloud. this means that seeing the mesh 
    //from the inside is still correct (instead of seeing the faces of the triangles when the normal is facing out)
    //one potential fix to this is to store the normal per vertex into a vector. 
    // before storing and observed point/vertex the direction of the frustum/UAV needs to be relatively in the opposite direction of the normal

private:
    OcclusionCulling* OC;
    UAV* drone;


    pcl::VoxelGridOcclusionEstimationT voxelGrid;
    float voxelRes, nb_occupiedVoxels, nb_freeVoxels;
    //bb indices
    Eigen::Vector3i max_b_idx, min_b_idx, gridSize;
    Eigen::Vector3f max_b, min_b;
    Eigen::Vector3i drone_start_index, drone_end_index;
    Eigen::Vector3i BB_start_index, BB_end_index;
    Eigen::Vector3f drone_start, drone_end;

    Graph graph;

    

public:

    PathPlanner();
	PathPlanner(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    ~PathPlanner();


    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> extract_surfaces_routine();

    void extract_surface(std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& map, Eigen::Vector4f new_position);
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> greedy_set_cover(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& final_viewpoints);
    std::pair<int, std::vector<pcl::PointXYZ> > calc_nb_intersection( pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>& seen_points);
    bool delete_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ point);

    /// <summary>
    /// Graph routines
    /// </summary>
    /// <param name="poly"></param>
    void construct_graph(Polyhedron& poly, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& viewpoints, std::vector<int>& viewpoint_graph_idx);
    void calculate_distances(std::vector<int>& viewpoint_graph_idx);
    void calculate_pair_distance(std::vector<int>& viewpoint_graph_idx);
};

