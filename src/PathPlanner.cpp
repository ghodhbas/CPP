#include "PathPlanner.h"
#include <iostream>

PathPlanner::PathPlanner() {
	std::cerr << "DO NOT USE THIS CONSTRUCTOR" << std::endl;
}

PathPlanner::PathPlanner(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
	OC = new OcclusionCulling(input_cloud);
	drone = new UAV();

	//create voxel grid of the point cloud
	voxelRes = 1.f;
	voxelGrid.setInputCloud(input_cloud);
	voxelGrid.setLeafSize(voxelRes, voxelRes, voxelRes);
	voxelGrid.initializeVoxelGrid();

	min_b_idx = voxelGrid.getMinBoxCoordinates();
    max_b_idx = voxelGrid.getMaxBoxCoordinates();
	Eigen::Vector3f leaf_size = voxelGrid.getLeafSize();

	min_b[0] = (static_cast<float> (min_b_idx[0]) * leaf_size[0]);
	min_b[1] = (static_cast<float> (min_b_idx[1]) * leaf_size[1]);
	min_b[2] = (static_cast<float> (min_b_idx[2]) * leaf_size[2]);
	max_b[0] = (static_cast<float> ((max_b_idx[0]) + 1) * leaf_size[0]);
	max_b[1] = (static_cast<float> ((max_b_idx[1]) + 1) * leaf_size[1]);
	max_b[2] = (static_cast<float> ((max_b_idx[2]) + 1) * leaf_size[2]);


	std::cout << "In Path Planner -  Voxel grid Min indicies: "<< min_b.x()<< ", "<< min_b.y()<<", "<< min_b.z() << std::endl;
	std::cout << "In Path Planner -  Voxel grid Max indicies: " << max_b.x() << ", " << max_b.y() << ", " << max_b.z() << std::endl;
	std::cout << "In Path Planner -  Voxel grid Min : " << min_b.x() << ", " << min_b.y() << ", " << min_b.z() << std::endl;
	std::cout << "In Path Planner -  Voxel grid Max : " << max_b.x() << ", " << max_b.y() << ", " << max_b.z() << std::endl;
	
	
	//// 3 and 5 is used to making the BB bigger not exactly on the boundry of the cluster
   //// (sometimes it is very small set of samples and the descritization sample will not fit)
   float maximizeSizeXY = 6.f;
   float maximizeSizeZ = 6.f;

   //increase the index and use it with voxel res to calculate position of the srone throughout the grid
   //with everyposition check if it's in the voxel grid, if it is che ck if it is occupied

	//gridSize[1] = std::abs(max_b_idx[1] - min_b_idx[1]) + maximizeSizeXY;//
	//gridSize[2] = std::abs(max_b_idx[2] - min_b_idx[2]) + maximizeSizeZ;//
	//gridSize[0] = std::abs(max_b_idx[0] - min_b_idx[0]) + maximizeSizeXY;//
	//////std::cout << "In Path Planner -  Drone indicies: " << gridSize[0] << ", " << gridSize[1] << ", " << gridSize[2] << std::endl;
	
	drone_start[0] = min_b[0] - ((float)maximizeSizeXY/2.f) ;//5
	drone_start[1] = min_b[1] - ((float)maximizeSizeXY/2.f) ;//5
	drone_start[2] = min_b[2] - ((float)maximizeSizeZ/2.f);//
	std::cout << "Drone start : " << drone_start << std::endl;

	drone_end[0] = max_b[0] + ((float)maximizeSizeXY / 2.f);//5
	drone_end[1] = max_b[1] + ((float)maximizeSizeXY / 2.f);//5
	drone_end[2] = max_b[2] + ((float)maximizeSizeZ / 2.f);//
	std::cout << "Drone end : " << drone_end << std::endl;

	drone_start_index[0] = min_b_idx[0] - ((int)(maximizeSizeXY / 2)/voxelRes);//5
	drone_start_index[1] = min_b_idx[1] - ((int)(maximizeSizeXY / 2)/voxelRes);//5
	drone_start_index[2] = min_b_idx[2] - ((int)(maximizeSizeZ / 2)/voxelRes);
	std::cout << "Drone start index: " << drone_start_index << std::endl;
	
	
	drone_end_index[0] = max_b_idx[0] + ((int)(maximizeSizeXY / 2)/voxelRes);
	drone_end_index[1] = max_b_idx[1] + ((int)(maximizeSizeXY / 2)/voxelRes);
	drone_end_index[2] = max_b_idx[2] + ((int)(maximizeSizeZ / 2) / voxelRes);
	std::cout << "Drone end index: " << drone_end_index << std::endl;

	
	
	BB_start_index[0] = min_b_idx[0] ;//5
	BB_start_index[1] = min_b_idx[1];//5
	BB_start_index[2] = min_b_idx[2];
	std::cout << "BB start index: " << BB_start_index << std::endl;


	BB_end_index[0] = max_b_idx[0] ;
	BB_end_index[1] = max_b_idx[1];
	BB_end_index[2] = max_b_idx[2];
	std::cout << "BB end index: " << BB_end_index << std::endl;
	
}	

PathPlanner::~PathPlanner() {
	delete OC;
	delete drone;
}

std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> PathPlanner::extract_surfaces_routine() {
	// map saves the position of the drone and the seen points fdrom that position
	std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> map;

	int i = 0;
	int nb_padding = 0;
	for (int kk = drone_start_index.z(); kk <= drone_end_index.z(); ++kk)
	{
		//std::cout << "z==> " << kk - drone_start_index.z() << " out of " << drone_end_index.z() - drone_start_index.z() << std::endl;
		for (int jj = drone_start_index.y(); jj <= drone_end_index.y(); ++jj)
		{
			//std::cout << "y==> " << jj - drone_start_index.y() << " out of " << drone_end_index.y() - drone_start_index.y() << std::endl;

			for (int ii = drone_start_index.x(); ii <= drone_end_index.x(); ++ii)
			{	
				//std::cout << "x==> " << ii - drone_start_index.x() << " out of " << drone_end_index.x() - drone_start_index.x() << std::endl;
				Eigen::Vector3i ijk1(ii, jj, kk);
				Eigen::Vector4f new_position = voxelGrid.getCentroidCoordinate(ijk1);

				//don't bather checking whether indices are in the padding (outside the BB)
				if (kk < BB_start_index.z() || kk> BB_end_index.z() || jj<BB_start_index.y() || jj > BB_end_index.y() || ii<BB_start_index.x() || ii > BB_end_index.x())
				{
					nb_padding++;
					extract_surface(map, new_position);

				}
				else {
					int index1 = voxelGrid.getCentroidIndexAt(ijk1);

					//std::cout << "Index: " << index1 << std::endl;
					//if occupied
					if (index1 != -1)
					{
						extract_surface(map, new_position);
						i++;

					}
				}

			}
		}
	}

	std::cout << "nb occupied: " << i << std::endl;
	std::cout << "nb Padding: " << nb_padding << std::endl;
	return map;
}

//fix the seen points to be a point cloud


void PathPlanner::extract_surface(std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& map, Eigen::Vector4f new_position) {

	//set position of the drone
	Eigen::Matrix4f pose = drone->get_pose();
	pose(0, 0) = 1.f;
	pose(2, 2) = 1.f;
	pose(0, 3) = new_position.x();
	pose(1, 3) = new_position.y();
	pose(2, 3) = new_position.z();
	//std::cout << pose << std::endl;
	drone->set_pose(pose);
	//extart surface from first angle
	pcl::PointCloud<pcl::PointXYZ> visibles_s = OC->extractVisibleSurface(*drone);
	if (visibles_s.points.size() > 0) {

		map.push_back(std::pair< Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>(pose, visibles_s));
		//std::cout << "TRUEE 1" << std::endl;
	}
	//extract surface from 2nd angle
		//set 2nd angle
	pose(0, 0) = -1.f;
	pose(2, 2) = -1.f;
	drone->set_pose(pose);
	//std::cout << pose << std::endl;
	visibles_s = OC->extractVisibleSurface(*drone);
	if (visibles_s.points.size() > 0) {
		//std::cout << "TRUEE 2" << std::endl;
		map.push_back(std::pair< Eigen::Matrix4f, pcl::PointCloud < pcl::PointXYZ>>(pose, visibles_s));
	}
}

