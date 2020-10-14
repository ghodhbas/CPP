#include "graph.h"

// A utility function to find the subset of an element i 
int find(std::map<int, int>& subsets, int i)
{
	if (subsets[i] == -1)
		return i;
	return find(subsets, subsets[i]);
}

// A utility function to do union of two subsets  
void Union(std::map<int, int>& subsets, int x, int y)
{
	int xset = find(subsets, x);
	int yset = find(subsets, y);
	if (xset != yset) {
		subsets[xset] = yset;
	}
}


int isCycle(Edge_Graph& graph) {
	//for the ith node -- save the parent
	std::map<int, int> subsets;
	//initilize subsets
	for (int i = 0; i < graph.node_idx.size(); i++)
	{
		subsets[graph.node_idx[i]] = -1;
	}

	for (size_t i = 0; i < graph.edges.size(); i++)
	{
		int x = find(subsets, graph.edges[i].src);
		int y = find(subsets, graph.edges[i].dest);
		if (x == y)
			return 1;

		Union(subsets, x, y);
	}

	return 0;
}