#pragma once
#include "Occlusion_Culling.h"
#include <vector>

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

public:

    PathPlanner();
	PathPlanner(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    ~PathPlanner();


    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> extract_surfaces_routine();

    void extract_surface(std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& map, Eigen::Vector4f new_position);



   /// getCentroidIndexAt(ijk) returen -1 if empty

    ////getting grid size and start
    //Eigen::Vector4f min_b = grid.getCentroidCoordinate(grid.getMinBoxCoordinates());
    //Eigen::Vector4f max_b = grid.getCentroidCoordinate(grid.getMaxBoxCoordinates());

   
    //1 - make a KDtree of the point cloud.
    //    2 - make clusters of the kdtree nodes
    //    3 - convert each cluster to a point cloud-- > now every kd node is a point cloud that can be used for surface extraction

    //    1 - voxelize each cluster and get bounding box                                                               //getting grid size and start
    //    Eigen::Vector4f min_b = grid.getCentroidCoordinate(grid.getMinBoxCoordinates());
    //Eigen::Vector4f max_b = grid.getCentroidCoordinate(grid.getMaxBoxCoordinates());
    //2 - add padding to the bounding box
    //    3 - select start of grid(starting voxel)

    //    1 - voxelize the surface into bigger blocks
    //    - padding
    //    - use centroids as  coordinate for drone
    //    - for each centroid sample 4 view point(1 looking in the x dir, and in the - x dir using 180 FOV)
    //    - add all results into a Vector4f
    //    - solve knapsack such that pick smallest number of viewpoints such that the number of vertices seen is ma



    //------------COVERED VOXELS OF THE MODEL-------------------------/


    // pcl::PointCloud<pcl::PointXYZ> coveredVoxels;
    //pcl::VoxelGridOcclusionEstimationT coveredCloudFilteredVoxels;
    //coveredCloudFilteredVoxels.setInputCloud(globalCloudPtr);
    //coveredCloudFilteredVoxels.setLeafSize(voxelResForConn, voxelResForConn, voxelResForConn);
    //coveredCloudFilteredVoxels.initializeVoxelGrid();
    //coveredCloudFilteredVoxels.filter(coveredVoxels);
    //std::cout << " ////// Coverage % : " << ((double)coveredVoxels.size() / (double)modelVoxelsForConn.size()) * 100 << " //////" << std::endl;

};

