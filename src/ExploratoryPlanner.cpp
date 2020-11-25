#include "ExploratoryPlanner.h"
#


namespace ExploratoryPlanner {

    float calculate_huristic(pcl::PointNormal currentpoint, pcl::PointNormal point, Eigen::Vector3f view_dir, pcl::PointNormal prevPoint) {
        //distance
        float d = pcl::euclideanDistance(currentpoint, point);

        //angle --TODO: add to huristic the angle between current view dir and next view dir?
        
        float angle;
        Eigen::Vector3f next_dir(point.x - currentpoint.x, point.y - currentpoint.y, point.z - currentpoint.z);
        next_dir.normalize();
        Eigen::Vector3f prev_dir(currentpoint.x-prevPoint.x,  currentpoint.y - prevPoint.y,  currentpoint.z - prevPoint.z);
        prev_dir.normalize();
        if (std::fabsf(prevPoint.x - currentpoint.x) < EPSILON && std::fabsf(prevPoint.y - currentpoint.y) < EPSILON && std::fabsf(prevPoint.z - currentpoint.z) < EPSILON) {

            //if first point the angle is between view dir and  x-z plane -- divide by norm if not normalized
            angle = asinf(next_dir.dot(Eigen::Vector3f(0.f, 1.f, 0.f)));
        }
        else {
            //divide by norm if not normalized
            angle = acosf(next_dir.dot(prev_dir));
        }
        angle = angle * (180.f / M_PI);

        //angle threshhold
        if(angle> 70.f) angle = angle *100;

        //coverage? -- instead of coverage in huristic we will cover all viewpoints that have been downsampeled

        return 1.f*angle+ 8.5f*d;
    }



    vector<Eigen::Vector3f >  generate_path(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, Viewpoints& viewpoints_list, SurfaceMesh& surface, Viewpoints& final_viewpoints) {

        vector<Eigen::Vector3f > path;

        //build map to mark if index is visited
        std::map<int, float> visited;
        for (size_t i = 0; i < downsampled_viewpoints->points.size(); i++)
        {
            visited[i] = false;
        }

        //choose starting point 
        int current_idx = 0;
        pcl::PointNormal currentPoint = downsampled_viewpoints->points[0];
        visited[current_idx] = true;
        int nb_visted = 1;
        pcl::PointNormal prevPoint = currentPoint;

        path.push_back(viewpoints_list[current_idx].first);

        // Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
        kdtree.setInputCloud(downsampled_viewpoints);
        float radius = 20.f; //search radius

        Tree tree(faces(surface).first, faces(surface).second, surface);
        


        while (nb_visted < downsampled_viewpoints->points.size()) {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            float min_cost = 999999999.f;
            int min_cost_idx = 9999999.f;

            //get neighbouring points within a certain distance
            if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                    //skip visited viewpoints
                    if (visited[pointIdxRadiusSearch[i]]) 
                        continue;

                    //if ( std::fabsf(point.x - currentPoint.x)< EPSILON && std::fabsf(point.y - currentPoint.y) < EPSILON && std::fabsf(point.z - currentPoint.z) < EPSILON)  continue;
                    //if (pointIdxRadiusSearch[i] == current_idx)continue;


                    pcl::PointNormal point = (*downsampled_viewpoints)[pointIdxRadiusSearch[i]];

                    //check if there is a collision between the two points -- ignore this point
                    if (MyMesh::ray_box_interstction(surface, Eigen::Vector3f(currentPoint.x, currentPoint.y, currentPoint.z), Eigen::Vector3f(point.x, point.y, point.z), tree))
                        continue;


                    //calculate hurestic for point
                    Eigen::Vector3f view_dir = viewpoints_list[pointIdxRadiusSearch[i]].second;
                    float cost = calculate_huristic(currentPoint, point, view_dir, prevPoint);
                    if (cost < min_cost) {
                        min_cost = cost;
                        min_cost_idx = i;
                    }

                    //std::cout << "    " << point.x
                    //    << " " << point.y
                    //    << " " << point.z
                    //    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                }


                //select i with the lowest huristic call
                if (min_cost_idx == 9999999.f) {
                    cout << "could not find another close unvisited point" << endl;
                    break;
                }
                ///update current
                prevPoint = currentPoint;
                currentPoint = (*downsampled_viewpoints)[pointIdxRadiusSearch[min_cost_idx]];
                visited[pointIdxRadiusSearch[min_cost_idx]] = true;
                path.push_back(viewpoints_list[pointIdxRadiusSearch[min_cost_idx]].first);
                final_viewpoints.push_back(viewpoints_list[pointIdxRadiusSearch[min_cost_idx]]);
                //increase number of points visited
                nb_visted++;
               



            }
            else
            {
                cout << "No neighbours near current point: ( " << currentPoint.x
                    << " " << currentPoint.y
                    << " " << currentPoint.z
                    << ")" << endl;
            }

          
            
        }

        return path;
	}





    void  generate_path_layer(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, Viewpoints& viewpoints_list, SurfaceMesh& surface, Viewpoints& final_viewpoints, Tree& tree, vector<Eigen::Vector3f >& final_path,  Eigen::Vector3f* last_point) {

        //build map to mark if index is visited
        std::map<int, float> visited;
        for (size_t i = 0; i < downsampled_viewpoints->points.size(); i++)
        {
            visited[i] = false;
        }

        //choose starting point -- if first layer it's null - else choose closest point to last layer
        int current_idx;
        pcl::PointNormal currentPoint;
        int nb_visted=1;
        pcl::PointNormal prevPoint;
        if (last_point == nullptr) {
            current_idx = 0;
            currentPoint = downsampled_viewpoints->points[0];
            prevPoint = currentPoint;
        }
        else {
            prevPoint = pcl::PointNormal((*last_point)[0], (*last_point)[1], (*last_point)[2]);
            float min_distance = 9999999999.f;
            //find closts point to the last point of prev layer
            int j = 0;
            int min_idx = 99999999.f;
            while(j < downsampled_viewpoints->points.size())
            {   
                pcl::PointNormal point = downsampled_viewpoints->points[j];
                
                float d = pcl::euclideanDistance(point, prevPoint);
                if (d < min_distance) {
                    //no collision
                    if (!MyMesh::ray_box_interstction(surface, Eigen::Vector3f(prevPoint.x, prevPoint.y, prevPoint.z), Eigen::Vector3f(point.x, point.y, point.z), tree)) {
                        min_distance = d;
                        min_idx = j;
                    }
                    
                }
                    j++;
            }
            current_idx = min_idx;
            currentPoint = downsampled_viewpoints->points[min_idx];
            
        }
        visited[current_idx] = true;

        final_path.push_back(viewpoints_list[current_idx].first);

        // Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
        kdtree.setInputCloud(downsampled_viewpoints);
        float radius = 1000.f; //search radius


        while (nb_visted < downsampled_viewpoints->points.size()) {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            float min_cost = 999999999.f;
            int min_cost_idx = 9999999.f;

            //get neighbouring points within a certain distance
            if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                    //skip visited viewpoints
                    if (visited[pointIdxRadiusSearch[i]]) 
                        continue;

                    //if ( std::fabsf(point.x - currentPoint.x)< EPSILON && std::fabsf(point.y - currentPoint.y) < EPSILON && std::fabsf(point.z - currentPoint.z) < EPSILON)  continue;
                    //if (pointIdxRadiusSearch[i] == current_idx)continue;


                    pcl::PointNormal point = (*downsampled_viewpoints)[pointIdxRadiusSearch[i]];

                    //check if there is a collision between the two points -- ignore this point
                    if (MyMesh::ray_box_interstction(surface, Eigen::Vector3f(currentPoint.x, currentPoint.y, currentPoint.z), Eigen::Vector3f(point.x, point.y, point.z), tree))
                        continue;


                    //calculate hurestic for point
                    Eigen::Vector3f view_dir = viewpoints_list[pointIdxRadiusSearch[i]].second;
                    float cost = calculate_huristic(currentPoint, point, view_dir, prevPoint);
                    if (cost < min_cost) {
                        min_cost = cost;
                        min_cost_idx = i;
                    }

                    //std::cout << "    " << point.x
                    //    << " " << point.y
                    //    << " " << point.z
                    //    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                }


                //select i with the lowest huristic call
                if (min_cost_idx == 9999999.f) {
                    cout << "could not find another close unvisited point" << endl;
                    break;
                }
                ///update current
                prevPoint = currentPoint;
                currentPoint = (*downsampled_viewpoints)[pointIdxRadiusSearch[min_cost_idx]];
                visited[pointIdxRadiusSearch[min_cost_idx]] = true;
                final_path.push_back(viewpoints_list[pointIdxRadiusSearch[min_cost_idx]].first);
                final_viewpoints.push_back(viewpoints_list[pointIdxRadiusSearch[min_cost_idx]]);
                //increase number of points visited
                nb_visted++;
               



            }
            else
            {
                cout << "No neighbours near current point: ( " << currentPoint.x
                    << " " << currentPoint.y
                    << " " << currentPoint.z
                    << ")" << endl;
            }

          
            
        }

	}
   
}
