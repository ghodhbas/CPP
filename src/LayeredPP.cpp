#include "LayeredPP.h"

LayeredPP::LayeredPP(float resolution) {
	voxelRes = resolution;
}


vector<std::pair<float, float>> LayeredPP::construct_cutoff_ranges(const int nb_layers, const std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max) 
{
	//USING Y-AXIS
	vector<std::pair<float, float> > ranges;
	//calculate increment
	float incr = (min_max.second.y - min_max.first.y) / (float)nb_layers;

	for (size_t i = 0; i < nb_layers; i++)
	{
		std::pair<pcl::PointXYZ, pcl::PointXYZ> range;

		float lower_limit = min_max.first.y;
		float upper_limit = min_max.first.y;
		lower_limit = min_max.first.y + i * incr;
		upper_limit = min_max.first.y + (i+1) * incr;

		ranges.push_back(std::pair<float, float>(lower_limit, upper_limit));
		//cout << "range: " << lower_limit << " --> " << upper_limit << endl;
	}


	//cout << endl;
	//cout << "LOWER LIM: " << min_max.first.y << endl;
	//cout << "Upper LIM: " << min_max.second.y << endl;
	return ranges;
}

//returns the layer in which the point is
int LayeredPP::get_range(pcl::PointNormal point, vector < std::pair<float, float>>& ranges) {
	for (size_t i = 0; i < ranges.size(); i++)
	{	
		std::pair<float, float> r = ranges[i];
		if (point.y >= r.first && point.y <= r.second) return i;
	}
}

void LayeredPP::classify_points_to_layers(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, vector<std::pair<float, float>>& ranges, vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layer_vec)
{
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		pcl::PointNormal point = cloud->points[i];
		int layer_idx = get_range(point, ranges);
		layer_vec[layer_idx]->points.push_back(point);
	}

	// Sanitation check
	//cout << "total nb of points " << cloud->points.size() << endl;
	//for (size_t i = 0; i < layer_vec.size(); i++)
	//{
	//	cout << "Layer " << i << " nb points --> " << layer_vec[i]->points.size() << endl;
	//}

}

vector< pcl::PointCloud<pcl::PointNormal>::Ptr> LayeredPP::construct_layers(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const int nb_layers, const std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max) {
	vector<std::pair<float, float>>ranges = construct_cutoff_ranges(nb_layers, min_max);
	vector< pcl::PointCloud<pcl::PointNormal>::Ptr>  layer_vec;
	//init vec
	for (size_t i = 0; i < nb_layers; i++)
	{
		layer_vec.push_back(pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>));
	}

	classify_points_to_layers(cloud, ranges, layer_vec);

	return layer_vec;
}





vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> LayeredPP::voxelize_layers(vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layers) {
	//to do occlusion and frustum culling 
	//create fc object
	//set input to the point cloud in the layer
	//  fc.setInputCloud(cloud);
	//fc.setVerticalFOV(30);
	//fc.setHorizontalFOV(30);
	//fc.setNearPlaneDistance(0.2);
	//fc.setFarPlaneDistance(2.0);
	// fc.setCameraPose(camera_pose);
	//fc.filter(*output);

	vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> voxel_layers;
	Eigen::Vector3f max_b, min_b;

	for (size_t i = 0; i < layers.size(); i++)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud = layers[i];
		pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> voxelGrid;
		//allow for normal downsampling with position
		voxelGrid.setDownsampleAllData(true);
		voxelGrid.setInputCloud(cloud);
		voxelGrid.setLeafSize(voxelRes, voxelRes, voxelRes);
		voxelGrid.initializeVoxelGrid();

		Eigen::Vector3i min_b_idx = voxelGrid.getMinBoxCoordinates();
		Eigen::Vector3i max_b_idx = voxelGrid.getMaxBoxCoordinates();
		Eigen::Vector3f leaf_size = voxelGrid.getLeafSize();
		

		min_b[0] = (static_cast<float> (min_b_idx[0]) * leaf_size[0]);
		min_b[1] = (static_cast<float> (min_b_idx[1]) * leaf_size[1]);
		min_b[2] = (static_cast<float> (min_b_idx[2]) * leaf_size[2]);
		max_b[0] = (static_cast<float> ((max_b_idx[0]) + 1) * leaf_size[0]);
		max_b[1] = (static_cast<float> ((max_b_idx[1]) + 1) * leaf_size[1]);
		max_b[2] = (static_cast<float> ((max_b_idx[2]) + 1) * leaf_size[2]);

		voxel_layers.push_back(voxelGrid);
		
		std::cout << "In Layered Planner -  Voxel grid Min indicies : " << min_b_idx.x() << ", " << min_b_idx.y() << ", " << min_b_idx.z() << std::endl;
		std::cout << "In Layered Planner -  Voxel grid Max indicies : " << max_b_idx.x() << ", " << max_b_idx.y() << ", " << max_b_idx.z() << std::endl;
		std::cout << "In Layer Planner -  Voxel grid Min: " << min_b.x() << ", " << min_b.y() << ", " << min_b.z() << std::endl;
		std::cout << "In Layer Planner -  Voxel grid Max: " << max_b.x() << ", " << max_b.y() << ", " << max_b.z() << std::endl << endl;
	}
	return voxel_layers;
}