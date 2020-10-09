#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <Eigen/Dense>

class Node {
public:
	std::vector<Node*> neighbours;
	Eigen::Vector4f position;
	Eigen::Vector3i index_vec;
	Eigen::Vector3i index_grid;
	int index_int;

	bool visited = false;
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
	return  vec.x() + gridSize[0] *  vec.y() + gridSize[0] * gridSize[1] * vec.z();
}
