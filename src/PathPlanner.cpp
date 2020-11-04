#include "PathPlanner.h"
#include <iostream>


PathPlanner::PathPlanner() {
	std::cerr << "DO NOT USE THIS CONSTRUCTOR" << std::endl;
}

PathPlanner::PathPlanner(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
	OC = new OcclusionCulling(input_cloud);
	drone = new UAV();

	//create voxel grid of the point cloud
	voxelRes = 25.f;
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


	std::cout << "In Path Planner -  Voxel grid Min indicies (without padding): "<< min_b_idx.x()<< ", "<< min_b_idx.y()<<", "<< min_b_idx.z() << std::endl;
	std::cout << "In Path Planner -  Voxel grid Max indicies (without padding): " << max_b_idx.x() << ", " << max_b_idx.y() << ", " << max_b_idx.z() << std::endl;
	std::cout << "In Path Planner -  Voxel grid Min (without padding): " << min_b.x() << ", " << min_b.y() << ", " << min_b.z() << std::endl;
	std::cout << "In Path Planner -  Voxel grid Max (without padding) : " << max_b.x() << ", " << max_b.y() << ", " << max_b.z() << std::endl;
	
	
	//// 3 and 5 is used to making the BB bigger not exactly on the boundry of the cluster
   //// (sometimes it is very small set of samples and the descritization sample will not fit)
   float maximizeSizeXY = 100.f;
   float maximizeSizeZ = 100.f;

   //increase the index and use it with voxel res to calculate position of the srone throughout the grid
   //with everyposition check if it's in the voxel grid, if it is che ck if it is occupied

	drone_start[0] = min_b[0] - ((float)maximizeSizeXY/2.f) ;//5
	//drone_start[1] = min_b[1] - ((float)maximizeSizeXY/2.f) ;//5
	drone_start[1] = min_b[1];//
	drone_start[2] = min_b[2] - ((float)maximizeSizeZ/2.f);//

	drone_end[0] = max_b[0] + ((float)maximizeSizeXY / 2.f);//5
	//drone_end[1] = max_b[1] + ((float)maximizeSizeXY / 2.f);//5
	drone_end[1] = max_b[1] + ((float)maximizeSizeXY );//5
	drone_end[2] = max_b[2] + ((float)maximizeSizeZ / 2.f);//
	

	drone_start_index[0] = min_b_idx[0] - (((int)(maximizeSizeXY / 2)) / voxelRes);
	//drone_start_index[1] = min_b_idx[1] - (((int)(maximizeSizeXY / 2)) / voxelRes);
	drone_start_index[1] = min_b_idx[1] ;
	drone_start_index[2] = min_b_idx[2] - (((int)(maximizeSizeXY / 2)) / voxelRes);;
	
	
	
	drone_end_index[0] = max_b_idx[0] + (((int)(maximizeSizeXY / 2))/voxelRes);
	//drone_end_index[1] = max_b_idx[1] + (((int)(maximizeSizeXY / 2))/voxelRes);
	drone_end_index[1] = max_b_idx[1] + (((int)(maximizeSizeXY)) / voxelRes);
	drone_end_index[2] = max_b_idx[2] + (((int)(maximizeSizeZ) / 2 )/ voxelRes);

	gridSize[0] = drone_end_index[0] - drone_start_index[0] +1;//
	gridSize[1] = drone_end_index[1] - drone_start_index[1] +1;//
	gridSize[2] = drone_end_index[2] - drone_start_index[2] +1;//

	std::cout << "In Path Planner -  Grid Size (with padding): " << gridSize[0] << ", " << gridSize[1] << ", " << gridSize[2] << std::endl;
	std::cout << "Drone start index: " << drone_start_index << std::endl;
	std::cout << "Drone end index: " << drone_end_index << std::endl;
	std::cout << "Drone start coordinates: " << drone_start << std::endl;
	std::cout << "Drone end coordinates: " << drone_end << std::endl;
	
	
	BB_start_index[0] = min_b_idx[0] ;//5
	BB_start_index[1] = min_b_idx[1];//5
	BB_start_index[2] = min_b_idx[2];
	//std::cout << "BB start index: " << BB_start_index << std::endl;


	BB_end_index[0] = max_b_idx[0] ;
	BB_end_index[1] = max_b_idx[1];
	BB_end_index[2] = max_b_idx[2];
	//std::cout << "BB end index: " << BB_end_index << std::endl;
	
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

				int index1 = voxelGrid.getCentroidIndexAt(ijk1);
				if (index1 == -1)
				{	
					//outside of shape within the padding
					if (kk < BB_start_index.z() || kk> BB_end_index.z() || jj<BB_start_index.y() || jj > BB_end_index.y() || ii<BB_start_index.x() || ii > BB_end_index.x())
					{
						nb_padding++;
						extract_surface(map, new_position);
						continue;

					}

					extract_surface(map, new_position);
					i++;

				}

				//don't bother checking whether indices are in the padding (outside the BB)
				

			}
		}
	}

	std::cout << "number of occupied voxels: " << i << std::endl;
	std::cout << "number Padding voxels: " << nb_padding << std::endl;
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



//fixing erasing donc advance 
bool PathPlanner::delete_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ point) {

	for (size_t cloud_idx = 0; cloud_idx < cloud->points.size(); cloud_idx++)
	{
		pcl::PointXYZ cloud_point = cloud->at(cloud_idx);
		if (std::abs(point.x - cloud_point.x) < EPSILON && std::abs(point.y - cloud_point.y) < EPSILON && std::abs(point.z - cloud_point.z) < EPSILON) {
			cloud->erase(cloud->begin() + cloud_idx);
			return true;
		}
	}
	return false;
}


std::pair<int, std::vector<pcl::PointXYZ> > PathPlanner::calc_nb_intersection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>& seen_points) {

	int nb_inter = 0;
	std::vector<pcl::PointXYZ> intersection_points;

	for (size_t seen_point_idx = 0; seen_point_idx < seen_points.size(); seen_point_idx++)
	{
		pcl::PointXYZ point = seen_points[seen_point_idx];
		for (size_t cloud_idx = 0; cloud_idx < cloud->points.size(); cloud_idx++)
		{
			pcl::PointXYZ cloud_point = cloud->at(cloud_idx);
			if (std::abs(point.x - cloud_point.x) < EPSILON && std::abs(point.y - cloud_point.y) < EPSILON && std::abs(point.z - cloud_point.z) < EPSILON) {
				nb_inter++;
				intersection_points.push_back(cloud_point);
				break;
			}
		}
	}


	return std::pair<int, std::vector<pcl::PointXYZ> >(nb_inter, intersection_points);
}



//Greedy - Set - Cover(X, S) {
//    U = X // U stores the uncovered items  ------ cloud
//        C = empty // C stores the sets of the cover
//        while (U is nonempty) {
//            select s[i] in S that covers the most elements of U---- -  select the one with  max intersection with U
//                add i to C
//                remove the elements of s[i] from U
//        }
//    return C
//}

std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> PathPlanner::greedy_set_cover(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& final_viewpoints) {
	//stores uncovered items
	pcl::PointCloud<pcl::PointXYZ>::Ptr uncovered = pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr covered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
	// stores the sets of the cover -- viewpoints that make the union
	std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> Solution;
	//stores the indecies of the solutions viewpoints
	std::vector<int> solution_indx;
	// contains number of intersction for the ith viewpoint
	//std::map<int, int> intersection_map;
	std::map<int, std::vector<pcl::PointXYZ>> to_remove_map;



	int solution_size = 0;
	int total_pts_removed = 0;

	//while we have not covered all the points on the cloud and did not try all viewpoints
	while ((uncovered->points.size() != 0 || covered->points.size() >= cloud->points.size()) && solution_size < final_viewpoints.size()) {

		int max_nb_intersections = 0;
		int index = -1;
		std::pair<int, std::vector<pcl::PointXYZ>> result;
		//select set that covers most of uncovered. viewpoint[j] inter uncovered is max
		for (int j = 0; j < final_viewpoints.size(); j++)
		{

			//if we have already selected this viewpoint move to the next one
			if (std::find(solution_indx.begin(), solution_indx.end(), j) != solution_indx.end()) continue;

			int nb_inter= 0;
			//calculate nb of intersections
			pcl::PointCloud<pcl::PointXYZ> seen_points = final_viewpoints[j].second;
			//get the intersection between what the camera sees and the cloud that has not been covered yet
			std::pair<int, std::vector<pcl::PointXYZ>> tmp = calc_nb_intersection(uncovered, seen_points);
			nb_inter = tmp.first;
			
			//found a new better viewpoint
			if (nb_inter > max_nb_intersections) {
				max_nb_intersections = nb_inter;
				index = j;
				result = tmp;
				//intersection_map.insert(std::pair<int, int>(index, nb_inter));
				//save the vertices that will be removed if this jth viewpoint is selected
			}
		}

		to_remove_map.insert(std::pair<int, std::vector<pcl::PointXYZ>>(index, result.second));
		
		if (index == -1) {
			std::cout << "Stopped looking for additional sets: No more intersections with the mesh" << std::endl;
			return Solution;
		}
		//add the set to soltion
		std::cout << "Best viewpoint index: " << index << std::endl;
		solution_indx.push_back(index);
		Solution.push_back(final_viewpoints[index]);
		//remove the seen points from uncovered
			//retrieve the indices of vertices to removes
		std::vector<pcl::PointXYZ> points_to_remove = to_remove_map[index];
		std::cout << "Cloud size before delete from viewpoint:   " << uncovered->points.size() << std::endl;
		std::cout << "Nb of points to remove: " << points_to_remove.size() << std::endl;

		//delete points from cloud using the selected viewpoint
		for (unsigned int i = 0; i < points_to_remove.size(); i++)
		{
			pcl::PointXYZ point = points_to_remove[i];
			covered->emplace_back(point);
			delete_point(uncovered, point);
			//total_pts_removed++;
		}
		std::cout << "Cloud size after delete from viewpoint: " << uncovered->points.size() << std::endl;
		std::cout << "covered size: " << covered->points.size() << std::endl << std::endl;
	}

	//fix viewed points
	cout << "solution index size"<< solution_indx.size() << endl;
	cout << "solution " << Solution.size() << endl;
	return Solution;

}


void PathPlanner::construct_graph(Polyhedron& poly, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& viewpoints, std::vector<int>& viewpoint_graph_idx) {
	//track index in graph
	int index = 0;

	std::vector<int> invalid_cell_idx;
	//map saves Key--> index in graph and  Value-->the the order of the cell in overall grid
	std::map<int, int> index_map;

	//used to check point inside of poly
	CGAL::Side_of_triangle_mesh<Polyhedron, Kernel> inside(poly);

	//track the index in the gris overall
	int count = 0;
	int xx = 0;

	vector<int> found;
	for (int kk = drone_start_index.z(); kk <= drone_end_index.z(); ++kk)
	{
		for (int jj = drone_start_index.y(); jj <= drone_end_index.y(); ++jj)
		{
			for (int ii = drone_start_index.x(); ii <= drone_end_index.x(); ++ii)
			{
				count++;
				Eigen::Vector3i ijk1(ii, jj, kk);
				Eigen::Vector4f new_position = voxelGrid.getCentroidCoordinate(ijk1);

				

				//skip if voxel occupied by verticies
				if (voxelGrid.getCentroidIndexAt(ijk1) != -1) {
					invalid_cell_idx.push_back(count-1);
					continue;
				}
					
				//skip if point inside mesh
				if (MyMesh::point_inside_mesh(inside, new_position)) {
					invalid_cell_idx.push_back(count-1);
					continue;
				}

				//shift grid to 0
				Eigen::Vector3i index_vec_shifted;
				index_vec_shifted[0] = ii - drone_start_index.x();
				index_vec_shifted[1] = jj - drone_start_index.y();
				index_vec_shifted[2] = kk - drone_start_index.z();
				
				//construct node
				Node* node = new Node();
				node->position = new_position;
				node->index_grid = ijk1;
				node->index_vec = index_vec_shifted;
				//std::cout << "Index_vec: " << node->index_vec << std::endl;
				node->index_int = index;

				index_map.emplace(std::pair<int, int>(count-1, index));


				//save indecies of graph where the viewpoints are
				for (int viewpoint_idx = 0; viewpoint_idx < viewpoints.size(); viewpoint_idx++) {
					Eigen::Matrix4f pose = viewpoints[viewpoint_idx].first;
					Eigen::Vector3f viewpoint_pos = pose.block(0, 3, 3, 1);

					//if (std::abs(new_position.x() - viewpoint_pos.x()) < EPSILON && std::abs(new_position.y() - viewpoint_pos.y()) < EPSILON && std::abs(new_position.z() - viewpoint_pos.z()) < EPSILON) {
					if (new_position.x()== viewpoint_pos.x()  && new_position.y() == viewpoint_pos.y() && new_position.z() == viewpoint_pos.z()) {

						//dont'add the same point twice (only to speed up graph search but there could be more than 1 viewpoint per location so Use Solution to get the entire POSE
						if (std::find(viewpoint_graph_idx.begin(), viewpoint_graph_idx.end(), index) == viewpoint_graph_idx.end()) {
							viewpoint_graph_idx.push_back(index);
							xx++;
							//found.push_back(viewpoint_idx);
						}
						//cout << "Found viewpoint to Grid index match: Viewpoint " << viewpoint_idx << " has position " << new_position << endl << " and index " << index << " in the graph." << endl << endl;
						
					}
				}

				//std::cout << "Testing conversion - int to vec: " << convert_int_to_vec(node->index_int, gridSize) << std::endl;
				//std::cout << "Index_int: " << node->index_int << std::endl;
				//std::cout << "Position: " << node->position << std::endl<< std::endl << std::endl;
				//std::cout << "Testing conversion - vec to index: " << convert_vec_to_int(node->index_vec, gridSize) << std::endl;
				
				graph.add_node(node);
				
				index++;
				
			}
			
		}
	}

	std::cout << "NB Nodes in graph: " << graph.nodes.size() << std::endl;
	std::cout << "NB of Unique viewpoint locations: "<<xx << endl<<endl;
	//std::cout << "loop count: " << count << std::endl;

	//save neighbours --assumin up is +t -- right +z -- front +x
	for (int i = 0; i < graph.size(); i++)
	{
		Node* node = graph[i];
		Eigen::Vector3i cell_vec = node->index_vec;
		int cell_idx = node->index_int;

		//save neighbor up   ---  check if it is valid and within bounds 
		Eigen::Vector3i up = cell_vec;
		up[1] = up[1] + 1;
		int up_idx = convert_vec_to_int(up, gridSize);
		//if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), up_idx) == invalid_cell_idx.end()) && up_idx >= 0 && up_idx < ((gridSize[0] * gridSize[1] * gridSize[2]) + 1)) {
		if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), up_idx) == invalid_cell_idx.end()) && up[0] >= 0 && up[1] >= 0 && up[2] >= 0 && up[0] < gridSize[0] && up[1] < gridSize[1] && up[2] < gridSize[2]) {
			//get index of the neighbour in the graph
			int index = index_map[up_idx];
			//save neighbour
			node->neighbours.push_back(graph[index]);
		}

		//save neighbor down
		Eigen::Vector3i down = cell_vec;
		down[1] = down[1] -1 ;
		int down_idx = convert_vec_to_int(down, gridSize);
		//if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), down_idx) == invalid_cell_idx.end()) && down_idx >= 0 && down_idx < ((gridSize[0] * gridSize[1] * gridSize[2]) +1)) {
		if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), down_idx) == invalid_cell_idx.end()) && down[0] >= 0 && down[1] >= 0 && down[2] >= 0 && down[0] < gridSize[0] && down[1] < gridSize[1] && down[2] < gridSize[2]) {
			//get index of the neighbour in the graph
			int index = index_map[down_idx];
			//save neighbour
			node->neighbours.push_back(graph[index]);
		}

		//save neighbor right
		Eigen::Vector3i right = cell_vec;
		right[2] = right[2] + 1;
		int right_idx = convert_vec_to_int(right, gridSize);
		//if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), right_idx) == invalid_cell_idx.end()) && right_idx >= 0 && right_idx < ((gridSize[0] * gridSize[1] * gridSize[2]) + 1)) {
		if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), right_idx) == invalid_cell_idx.end()) && right[0] >= 0 && right[1] >= 0 && right[2] >= 0 && right[0] < gridSize[0] && right[1] < gridSize[1] && right[2] < gridSize[2]) {
			//get index of the neighbour in the graph
			int index = index_map[right_idx];
			//save neighbour
			node->neighbours.push_back(graph[index]);
		}

		//save neighbor left
		Eigen::Vector3i left = cell_vec;
		left[2] = left[2] - 1;
		int left_idx = convert_vec_to_int(left, gridSize);
		//if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), left_idx) == invalid_cell_idx.end()) && left_idx >= 0 && left_idx < ((gridSize[0] * gridSize[1] * gridSize[2]) + 1)) {
		if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), left_idx) == invalid_cell_idx.end()) && left[0] >= 0 && left[1] >= 0 && left[2] >= 0 && left[0] < gridSize[0] && left[1] < gridSize[1] && left[2] < gridSize[2]) {
		//get index of the neighbour in the graph
			int index = index_map[left_idx];
			//save neighbour
			node->neighbours.push_back(graph[index]);
		}

		//save neighbor front
		Eigen::Vector3i  front= cell_vec;
		front[0] = front[0] + 1;
		int front_idx = convert_vec_to_int(front, gridSize);
		//if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), front_idx) == invalid_cell_idx.end()) && front_idx >= 0 && front_idx < ((gridSize[0] * gridSize[1] * gridSize[2]) + 1)) {
		if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), front_idx) == invalid_cell_idx.end()) && front[0] >= 0 && front[1] >= 0 && front[2] >= 0 && front[0] < gridSize[0] && front[1] < gridSize[1] && front[2] < gridSize[2]) {
		//get index of the neighbour in the graph
			int index = index_map[front_idx];
			//save neighbour
			node->neighbours.push_back(graph[index]);
		}

		//save neighbor back
		Eigen::Vector3i  back = cell_vec;
		back[0] = back[0] - 1;
		int back_idx = convert_vec_to_int(back, gridSize);
		//check that we dont exceed bounds and that cell is valid
		if ((std::find(invalid_cell_idx.begin(), invalid_cell_idx.end(), back_idx) == invalid_cell_idx.end()) && back[0] >= 0 && back[1] >= 0 && back[2] >= 0 && back[0] < gridSize[0] && back[1] < gridSize[1] && back[2] < gridSize[2]) {
			//get index of the neighbour in the graph
			int index = index_map[back_idx];
			//save neighbour
			node->neighbours.push_back(graph[index]);
		}


		//add diagnals?
	}

}



void PathPlanner::combinations_recursive(const vector<int>& elems, unsigned long req_len,
	vector<unsigned long>& pos, unsigned long depth,
	unsigned long margin, vector<std::pair<int, int>>& pair_vec)
{
	// Have we selected the number of required elements?
	if (depth >= req_len) {
		std::pair<int, int> pair(elems[pos[0]], elems[pos[1]]);
		pair_vec.push_back(pair);
		return;
	}

	// Are there enough remaining elements to be selected?
	// This test isn't required for the function to be correct, but
	// it can save a good amount of futile function calls.
	if ((elems.size() - margin) < (req_len - depth))
		return;

	// Try to select new elements to the right of the last selected one.
	for (unsigned long ii = margin; ii < elems.size(); ++ii) {
		pos[depth] = ii;
		combinations_recursive(elems, req_len, pos, depth + 1, ii + 1, pair_vec);
	}
	return;
}

void PathPlanner::combinations(const vector<int>& elems, vector<std::pair<int, int>>& pair_vec)
{
	vector<unsigned long> positions(2, 0);
	combinations_recursive(elems, 2, positions, 0, 0, pair_vec);
}

int PathPlanner::calculate_shortest_distance(int index_1, int index_2) {
	//intialiaze all nodes to not visited
	for (int i = 0; i < graph.size(); i++)
	{
		graph[i]->visited = false;
	}

	//stores the distance to the ith index in the graph -- intialize
	std:vector<int> distances;
	for (int i = 0; i < graph.size(); i++) {
		if (i == index_1) distances.push_back(0);
		else distances.push_back(INT_MAX-2);
	}
	
	std::queue<Node*> queue;
	queue.push(graph[index_1]);

	while (!queue.empty())
	{
		Node* node = queue.front();
		queue.pop();
		node->visited = true;
		//found goal destination 
		if (node->index_int == index_2) break;


		for (int i = 0; i < node->neighbours.size(); i++) {
			Node* neighbour = node->neighbours[i];
			if (neighbour->visited) {
				continue;
			}
			//update distance
			if (distances[neighbour->index_int] > (distances[node->index_int] + 1)) {
				distances[neighbour->index_int] = distances[node->index_int] + 1;
				
				queue.push(neighbour);
			}
		}


	}
	
	return distances[index_2];
}


std::map<int, std::map<int, int>> PathPlanner::calculate_distances(std::vector<int>& viewpoint_graph_idx, vector<std::pair<int, int>>& pair_vec, SurfaceMesh& surface) {
	//get the indecis of the viewpoints in the gris and in the graph

	//key: index of source in the graph
	// value map: distance to other cells in the graph	
	std::map<int,std::map<int,int>> distance_map;

	
	combinations(viewpoint_graph_idx, pair_vec);



	for (int i = 0; i < pair_vec.size(); i++)
	{
	
		//cout << "PAIR: " << pair_vec[i].first << ", " << pair_vec[i].second << endl;
		int distance = calculate_shortest_distance(pair_vec[i].first, pair_vec[i].second);
		//cout << "Shortest distance = " << distance << endl << endl;
		
		//save distance from one cell to the other
		if (distance == (INT_MAX - 2)) {

			distance_map[pair_vec[i].first][pair_vec[i].second] = distance ;
			distance_map[pair_vec[i].second][pair_vec[i].first] = distance ;
		}
		else {
			distance_map[pair_vec[i].first][pair_vec[i].second] = distance * voxelRes;
			distance_map[pair_vec[i].second][pair_vec[i].first] = distance * voxelRes;
		}

	}
	//cout << "nb pairs: " << pair_vec.size() << endl<<endl;


	//distance map can also be seen as an edge map with duplicates
	return distance_map;
}



//kruskal MST --	1. Sort all the edges in non-decreasing order of their weight.
				  //2. Pick the smallest edge.Check if it forms a cycle with the spanning tree formed so far.If cycle is not formed, include this edge.Else, discard it.
				  //3. Repeat step#2 until there are(V - 1) edges in the spanning tree.
Edge_Graph PathPlanner::construct_MST(vector<std::pair<int, int>>& pair_vec, std::map<int, std::map<int, int>>& distance_map) {
	int total_distance = 0;
	//spanning tree
	Edge_Graph MST = Edge_Graph(distance_map.size());
	
	//construct complete edge graph that will be the input for the MST
	//V nb of nodes, E = V(V-1)/2
	Edge_Graph e_graph = Edge_Graph(distance_map.size());

	// TO DO -- chaneg this to insert in corrrect order for speedup
	//add edges to the graph
	for (int i = 0; i < pair_vec.size(); i++)
	{	
		e_graph.add_edge(pair_vec[i].first, pair_vec[i].second, distance_map[pair_vec[i].first][pair_vec[i].second] );
		
	}
	//sort edges longfest to shortest
	std::sort(e_graph.edges.begin(), e_graph.edges.end(), Edge_Comparator());

	int idx = e_graph.edges.size() - 1;
	while (MST.edges.size() < (distance_map.size() -1)) {
		Edge e = e_graph.edges[idx];

		//temporary graph to test if adding the edge creates a cycle
		Edge_Graph tmp = Edge_Graph(MST);
		tmp.add_edge(e);

		//test if cycle is not created -- this can be optimized by incorporating subset and union here instead of contructing it every time in is cycle
		if (!isCycle(tmp)) {
			//add the edge to the MST
			MST.add_edge(e);
			//weight time cube size
			total_distance += e.weight;
		}

		idx--;
	}

	cout << "LENGTH OF MST: " << total_distance << endl;

	return MST;
}


void PathPlanner::DFS(vector<Eigen::Vector3f>& path, Node* node) {
	node->visited = true;

	Eigen::Vector3f position = node->position.block(0, 0, 3, 1);
	path.push_back(position);

	for (size_t i = 0; i < node->neighbours.size(); i++)
	{
		if (!node->neighbours[i]->visited) DFS(path, node->neighbours[i]);
	}

}

vector<Eigen::Vector3f> PathPlanner::generate_path(Edge_Graph& MST) {
	//construct vertex based graph from MST
	Graph MST_g;

	//map index of node to the index in the MST_g
	std::map<int, int> map_gidx_idx;
	//pick vertex with lowest  y value(closest to ground)
	for (size_t i = 0; i < MST.node_idx.size(); i++)
	{	
		int index = MST.node_idx[i];
		Node* n = new Node(*graph.nodes[index]);
		n->neighbours.clear();
		n->visited = false;
		MST_g.add_node(n);
		
		map_gidx_idx[n->index_int] = i;
		
	}


	Node* chosen = MST_g.nodes[0];
	//save smalled index;
	for (size_t i = 1; i < MST_g.nodes.size(); i++)
	{	
		Node* n = MST_g.nodes[i];
		if (chosen->position.y() > n->position.y()) chosen = n;
	}

	//save neighbour;
	for (size_t i = 0; i < MST.edges.size(); i++)
	{
		Edge e = MST.edges[i];
		int src = e.src;
		int dst = e.dest;

		Node* src_n = MST_g.nodes[map_gidx_idx[src]];
		Node* dst_n = MST_g.nodes[map_gidx_idx[dst]];

		src_n->neighbours.push_back(dst_n);
		dst_n->neighbours.push_back(src_n);
	}

	//vector<int> path;
	vector< Eigen::Vector3f> path;

	DFS(path, chosen);

	return path;
}