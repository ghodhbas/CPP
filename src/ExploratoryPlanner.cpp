#include "ExploratoryPlanner.h"



namespace ExploratoryPlanner {

    float calculate_huristic(pcl::PointNormal currentpoint, pcl::PointNormal point) {
        //distance


        //angle



        //coverage? -- instead of coverage in huristic we will cover all viewpoints that have been downsampeled

        return 0.f;
    }



	void generate_path(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, SurfaceMesh& surface) {

        //build map to mark if index is visited
        std::map<int, float> visited;
        for (size_t i = 0; i < downsampled_viewpoints->points.size(); i++)
        {
            visited[i] = false;
        }

        //choose starting point 
        int current_idx = 0;
        pcl::PointNormal currentPoint = downsampled_viewpoints->points[0];
        int nb_visted = 1;

        // Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
        kdtree.setInputCloud(downsampled_viewpoints);
        float radius = 2.f; //search radius

        Tree tree(faces(surface).first, faces(surface).second, surface);
        


        while (nb_visted < downsampled_viewpoints->points.size()) {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            std::vector<float> cost_list;

            //get neighbouring points within a certain distance
            if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                    //skip visited viewpoints
                    if (visited[pointIdxRadiusSearch[i]]) 
                        continue;


                    pcl::PointNormal point = (*downsampled_viewpoints)[pointIdxRadiusSearch[i]];

                    //check if there is a collision between the two points -- ignore this point
                    if (MyMesh::ray_box_interstction(surface, Eigen::Vector3f(currentPoint.x, currentPoint.y, currentPoint.z), Eigen::Vector3f(point.x, point.y, point.z), tree))
                        continue;


                    //calculate hurestic for point
                    cost_list.push_back(calculate_huristic(currentPoint, point));

                    //std::cout << "    " << point.x
                    //    << " " << point.y
                    //    << " " << point.z
                    //    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                }
            }
            else
            {
                cout << "No neighbours near current point: ( " << currentPoint.x
                    << " " << currentPoint.y
                    << " " << currentPoint.z
                    << ")" << endl;
            }

          
            //update current
            //select i with the lowest huristic call

            //increase number of points visited
            //set visted node to true
        }

        
	}
}
