#include "LayeredPP.h"

LayeredPP::LayeredPP( float near, float far) {
	near_plane_d = near;
	far_plane_d = far;
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
	return ranges.size() - 1;
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

pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> LayeredPP::voxelize(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, float voxelRes) {
	Eigen::Vector3f max_b, min_b;

	pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> voxelGrid;
	//allow for normal downsampling with position
	voxelGrid.setDownsampleAllData(true);
	voxelGrid.setInputCloud(cloud);
	voxelGrid.setLeafSize(voxelRes, voxelRes, voxelRes);
	voxelGrid.initializeVoxelGrid();

	Eigen::Vector3i min_b_idx = voxelGrid.getMinBoxCoordinates();
	Eigen::Vector3i max_b_idx = voxelGrid.getMaxBoxCoordinates();
	Eigen::Vector3f leaf_size = voxelGrid.getLeafSize();


	//min_b[0] = (static_cast<float> (min_b_idx[0]) * leaf_size[0]);
	//min_b[1] = (static_cast<float> (min_b_idx[1]) * leaf_size[1]);
	//min_b[2] = (static_cast<float> (min_b_idx[2]) * leaf_size[2]);
	//max_b[0] = (static_cast<float> ((max_b_idx[0]) + 1) * leaf_size[0]);
	//max_b[1] = (static_cast<float> ((max_b_idx[1]) + 1) * leaf_size[1]);
	//max_b[2] = (static_cast<float> ((max_b_idx[2]) + 1) * leaf_size[2]);
	//
	//std::cout << "In Layered Planner -  Voxel grid Min indicies : " << min_b_idx.x() << ", " << min_b_idx.y() << ", " << min_b_idx.z() << std::endl;
	//std::cout << "In Layered Planner -  Voxel grid Max indicies : " << max_b_idx.x() << ", " << max_b_idx.y() << ", " << max_b_idx.z() << std::endl;
	//std::cout << "In Layer Planner -  Voxel grid Min: " << min_b.x() << ", " << min_b.y() << ", " << min_b.z() << std::endl;
	//std::cout << "In Layer Planner -  Voxel grid Max: " << max_b.x() << ", " << max_b.y() << ", " << max_b.z() << std::endl << endl;

	return voxelGrid;

}



vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> LayeredPP::voxelize_layers(vector< pcl::PointCloud<pcl::PointNormal>::Ptr>& layers, float voxelRes) {
	

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
		
		//std::cout << "In Layered Planner -  Voxel grid Min indicies : " << min_b_idx.x() << ", " << min_b_idx.y() << ", " << min_b_idx.z() << std::endl;
		//std::cout << "In Layered Planner -  Voxel grid Max indicies : " << max_b_idx.x() << ", " << max_b_idx.y() << ", " << max_b_idx.z() << std::endl;
		//std::cout << "In Layer Planner -  Voxel grid Min: " << min_b.x() << ", " << min_b.y() << ", " << min_b.z() << std::endl;
		//std::cout << "In Layer Planner -  Voxel grid Max: " << max_b.x() << ", " << max_b.y() << ", " << max_b.z() << std::endl << endl;
	}
	return voxel_layers;
}



std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> LayeredPP::generate_viewpoints(pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> layer)
{
	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> result;

	pcl::PointCloud<pcl::PointNormal> filtered_cloud = layer.getFilteredPointCloud();

	for (size_t i = 0; i < filtered_cloud.points.size(); i++)
	{
		pcl::PointNormal point = filtered_cloud.points.at(i);
		Eigen::Vector3f position = Eigen::Vector3f(point.x, point.y, point.z);
		Eigen::Vector3f normal = Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z);

		//get new viewpoint location by starting from the centroid (point) going in the direction of the normal enough for UAV to see (a little less then max view distance)
		Eigen::Vector3f viewpoint_pos = position + ((far_plane_d-distance_epsilon)*normal);
		//get look direction  -- oposite of notmal
		Eigen::Vector3f look_dir = -normal;
		look_dir.normalize();

		//TODO double check that this location is free before adding it?

		result.push_back(std::pair<Eigen::Vector3f, Eigen::Vector3f>(viewpoint_pos, look_dir));
	}

	return result;
}


void LayeredPP::output_viewpoints(std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>>& layer_viewpoints) {
	for (size_t j = 0; j < layer_viewpoints.size(); j++)
	{
		std::vector< Kernel::Point_3> output_viewpoints_cloud;
		for (size_t i = 0; i < layer_viewpoints[j].size(); i++)
		{
			Eigen::Vector3f coord = layer_viewpoints[j][i].first;
			output_viewpoints_cloud.push_back(Kernel::Point_3(coord.x(), coord.y(), coord.z()));
		}
		//output point cloud for viewpoints
		IO::write_PLY("out/viewpoints_" + std::to_string(j) + ".ply", output_viewpoints_cloud);
	}

}






void LayeredPP::combinations_recursive(const vector<int>& elems, unsigned long req_len,
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



void LayeredPP::combinations(const vector<int>& elems, vector<std::pair<int, int>>& pair_vec)
{
	vector<unsigned long> positions(2, 0);
	combinations_recursive(elems, 2, positions, 0, 0, pair_vec);
}


std::map<int, std::map<int, float>> LayeredPP::calculate_distances(std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& viewpoints, vector<std::pair<int, int>>& pair_vec, SurfaceMesh& surface) {
	//get the indecis of the viewpoints in the gris and in the graph
	cout << "NB viewpoints in layer: " << viewpoints.size() << endl;
	//key: index of source in the graph
	// value map: distance to other cells in the graph	
	std::map<int, std::map<int, float>>  distance_map;

	std::vector<int> viewpoint_graph_idx(viewpoints.size());
	std::iota(viewpoint_graph_idx.begin(), viewpoint_graph_idx.end(), 0);
	combinations(viewpoint_graph_idx, pair_vec);

	int i = 0;

	//for (int i = 0; i < pair_vec.size(); i++)
	//{
	while (i < pair_vec.size())
	{
		//cout << "PAIR: " << pair_vec[i].first << ", " << pair_vec[i].second << endl;
		Eigen::Vector3f p1 = viewpoints[pair_vec[i].first].first;
		Eigen::Vector3f p2 = viewpoints[pair_vec[i].second].first;

		//use ray box inter
		
		Eigen::Vector3f Hit;
		CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(surface);
		//if(MyMesh::checkLineBox(Eigen::Vector3f(bbox.xmin(), bbox.ymin(),bbox.zmin()) , Eigen::Vector3f(bbox.xmax(), bbox.ymax(), bbox.zmax()), p1,p2,  Hit)){
		if (MyMesh::ray_box_interstction(surface, p1, p2)) {
		//if (MyMesh::intersect(p1,p2,bbox)) {

			//remove this pair because it is not valid
			auto idx = pair_vec.begin();
			std::advance(idx, i);
			pair_vec.erase(idx);
			//can add actual distance calculation (ex diskjtra)
			//distance_map[pair_vec[i].first][pair_vec[i].second] = 999999999999.f;
			//distance_map[pair_vec[i].second][pair_vec[i].first] = 999999999999.f;
			
		}else {
			float d = sqrtf(powf(p2[0] - p1[0], 2) + powf(p2[1] - p1[1], 2) + powf(p2[2] - p1[2], 2));
			//cout << "Shortest distance = " << d << endl << endl;
			//save distance from one cell to the other
			distance_map[pair_vec[i].first][pair_vec[i].second] = d;
			distance_map[pair_vec[i].second][pair_vec[i].first] = d;
			i++;
		}
		
	}
	cout << "nb pairs: " << pair_vec.size() << endl;
	
	
	//distance map can also be seen as an edge map with duplicates
	return distance_map;
}



//kruskal MST --	1. Sort all the edges in non-decreasing order of their weight.
				  //2. Pick the smallest edge.Check if it forms a cycle with the spanning tree formed so far.If cycle is not formed, include this edge.Else, discard it.
				  //3. Repeat step#2 until there are(V - 1) edges in the spanning tree.
Edge_Graph LayeredPP::construct_MST(vector<std::pair<int, int>>& pair_vec, std::map<int, std::map<int, float>>& distance_map){
	//spanning tree
	Edge_Graph MST = Edge_Graph(distance_map.size());

	//construct complete edge graph that will be the input for the MST
	//V nb of nodes, E = V(V-1)/2
	Edge_Graph e_graph = Edge_Graph(distance_map.size());

	// TO DO -- change this to insert in corrrect order for speedup
	//add edges to the graph
	for (int i = 0; i < pair_vec.size(); i++)
	{
		e_graph.add_edge(pair_vec[i].first, pair_vec[i].second, distance_map[pair_vec[i].first][pair_vec[i].second]);

	}
	//sort edges longest to shortest
	std::sort(e_graph.edges.begin(), e_graph.edges.end(), Edge_Comparator());

	int idx = e_graph.edges.size() - 1;
	while (MST.edges.size() < (distance_map.size() - 1)) {
		Edge e = e_graph.edges[idx];
	
		//temporary graph to test if adding the edge creates a cycle
		Edge_Graph tmp = Edge_Graph(MST);
		tmp.add_edge(e);
	
		//test if cycle is not created -- this can be optimized by incorporating subset and union here instead of contructing it every time in is cycle
		if (!isCycle(tmp)) {
			//add the edge to the MST
			MST.add_edge(e);
		}
	
		idx--;
	}

	return MST;
}



//TODO: FIX / Improve?
void LayeredPP::DFS(vector<Eigen::Vector3f>& path, Node* node, Node* prev) {
	node->visited = true;
	Eigen::Vector3f position = node->position.block(0, 0, 3, 1);
	path.push_back(position);
	

	for (size_t i = 0; i < node->neighbours.size(); i++)
	{	

		if (!node->neighbours[i]->visited) {
			DFS(path, node->neighbours[i], node);
		}
	
	}
	//Eigen::Vector3f position = node->position.block(0, 0, 3, 1);
	//path.push_back(position);
	//if (prev != nullptr) {
	//	//if number is written then don
	//	Eigen::Vector3f prev_position = z->position.block(0, 0, 3, 1);
	//	path.push_back(prev_position);
	//	prev->visited = false;
	//}
	


}

vector<Eigen::Vector3f> LayeredPP::generate_path(Edge_Graph& MST, std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& layer, Eigen::Vector3f& last_node) {
	//construct vertex based graph from MST
	Graph MST_g;

	//map index of node to the index in the MST_g
	std::map<int, int> map_gidx_idx;
	for (size_t i = 0; i < MST.node_idx.size(); i++)
	{
		int index = MST.node_idx[i];
		Node* n = new Node();
		n->position = Eigen::Vector4f(layer[index].first[0], layer[index].first[1], layer[index].first[2],0.f);
		n->neighbours.clear();
		n->visited = false;
		MST_g.add_node(n);

		map_gidx_idx[index] = i;

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



	Node* chosen;

	if ( last_node[0]== 0.f && last_node[1] == 0.f  && last_node[2]==0.f) {
		//pick vertex with lowest  y value(closest to ground)
		chosen = MST_g.nodes[0];
		for (size_t i = 1; i < MST_g.nodes.size(); i++)
		{
			Node* n = MST_g.nodes[i];
			if (chosen->position.y() > n->position.y()) chosen = n;
		}
		//srand(time(NULL));
		//chosen = MST_g.nodes[rand() % MST_g.nodes.size()];
	}
	else {
		//calculate distance from the set of points to last_viewpoint and pick closes
		vector<float> d_vec;
		for (size_t i = 0; i < MST_g.nodes.size(); i++)
		{
			Node* n = MST_g.nodes[i];
			Eigen::Vector3f pos = n->position.block(0, 0, 3, 1);
			float d = (last_node - pos).norm();
			d_vec.push_back(d);
			//cout << "i: " << i << " -- " << d << endl;
		}
		auto smallest = std::min_element(d_vec.begin(), d_vec.end());
		if (smallest == d_vec.end()) {
			cerr << "PATH GENERATION: COULDNT FIND CLOSEST to last node POINT to start the path";
		}
		int idx = std::distance(d_vec.begin(), smallest);
		chosen = MST_g.nodes[idx];
	}



	//vector<int> path;
	vector< Eigen::Vector3f> path;

	DFS(path, chosen, nullptr);

	cout << "path size: " << path.size() << endl;
	cout << "nb nodes: " << MST_g.nodes.size() << endl<<endl;

	return path;
}
