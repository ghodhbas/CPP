#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <map>


/***************** VOXEL GRID GRAPH**************************/
class Node {
public:
	std::vector<Node*> neighbours;
	Eigen::Vector4f position;
	Eigen::Vector3i index_vec;
	Eigen::Vector3i index_grid;
	int index_int;

	bool visited = false;

	Node() {
	}

	Node(const Node& n) {
		neighbours = n.neighbours;
		position = n.position;
		index_vec = n.index_vec;
		index_grid = n.index_grid;
		index_int = n.index_int;
	}
};


class Graph {
public:
	std::vector<Node*> nodes;
	void add_node(Node* n) { nodes.push_back(n); }

	int size() { return nodes.size(); }
	Node* operator[](const int i) { return nodes[i]; }
};



static Eigen::Vector3i convert_int_to_vec(int idx, Eigen::Vector3i gridSize) {
	int x = idx % gridSize.x();
	int y = (idx / gridSize.x()) % gridSize.y();
	int z = idx / (gridSize.x() * gridSize.y());

	return Eigen::Vector3i(x, y, z);
}


static int convert_vec_to_int(Eigen::Vector3i vec, Eigen::Vector3i gridSize) {
	return  vec.x() + gridSize[0] * vec.y() + gridSize[0] * gridSize[1] * vec.z();
}



//------------------------------------------------EDGE BASED GRAPH----------------------------------------/
class Edge {
public:
	int src, dest, weight;

	Edge(int s, int d, int w) { 
		src = s; 
		dest = d;
		weight = w;
	}

	
};

class Edge_Comparator {
public:
	bool operator() (const Edge& e1, const Edge& e2) {
		return e1.weight > e2.weight;
	}
};

class Edge_Graph {
public:
	
	//vertices by index in voxel graph
	std::vector<int> node_idx;

	// graph is represented as an array of edges
	std::vector<Edge> edges;

	Edge_Graph(int V) { 
	}

	Edge_Graph(const Edge_Graph& graph) {

		for (size_t i = 0; i < graph.edges.size(); i++)
		{
			add_edge(graph.edges[i]);
		}
	}

	void add_edge(int src, int dst, int w) {
		edges.push_back(Edge(src, dst,w ));
		if (std::find(node_idx.begin(), node_idx.end(), src) == node_idx.end()) node_idx.push_back(src);
		if (std::find(node_idx.begin(), node_idx.end(), dst) == node_idx.end()) node_idx.push_back(dst);
	}

	void add_edge(Edge e) {
		//edges.push_back(Edge(src, dst, w));
		edges.push_back(e);
		if (std::find(node_idx.begin(), node_idx.end(), e.src) == node_idx.end()) node_idx.push_back(e.src);
		if (std::find(node_idx.begin(), node_idx.end(), e.dest) == node_idx.end()) node_idx.push_back(e.dest);
	}
};


/*----------------------------- EDGE GRAPH METHODS------------------------------------------------------*/

// A utility function to find the subset of an element i 
int find(std::map<int, int>& subsets, int i);

// A utility function to do union of two subsets  
void Union(std::map<int, int>& subsets, int x, int y);

int isCycle(Edge_Graph& graph);