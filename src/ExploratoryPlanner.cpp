#include "ExploratoryPlanner.h"
#


namespace ExploratoryPlanner {

    float calculate_huristic(pcl::PointNormal currentpoint, pcl::PointNormal point, float cov, Eigen::Vector3f view_dir, pcl::PointNormal prevPoint) {
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
        if(angle> 130.f) angle = angle *100;



        //coverage? -- instead of coverage in huristic we will cover all viewpoints that have been downsampeled



        //return  10.f * d +  angle - cov;
        return  10.f * d  - cov;
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
        float radius = 30.f; //search radius

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
                    float cost = calculate_huristic(currentPoint, point, 0.f, view_dir, prevPoint);
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





    void  generate_path_layer(pcl::PointCloud<pcl::PointNormal>::Ptr& downsampled_viewpoints, Viewpoints& viewpoints_list, pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud, float near, float far, float Hfov, float Vfov, float voxelRes, float curr_radius, SurfaceMesh& surface, Viewpoints& final_viewpoints, Tree& tree, vector<Eigen::Vector3f >& final_path,  Eigen::Vector3f* last_point ) {

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
        float radius = curr_radius; //search radius




        //calculate coverage 
        // calculate the coverage for viewpoints - to be used in huristic
        vector<int> coverage_per_viewpoint = calculate_coverage(downsampled_viewpoints, voxel_cloud, near, far, Hfov, Vfov, voxelRes);

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
                    //caluculate coverage 
                    float cov = coverage_per_viewpoint[pointIdxRadiusSearch[i]];
                    float cost = calculate_huristic(currentPoint, point, cov, view_dir, prevPoint);
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



    vector<int> calculate_coverage(pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_viewpoints, pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud, float near, float far, float Hfov, float Vfov, float voxelRes) {
        

        

        vector<int> nb_coverage;
        // for every viewpoint
        for (int i = 0; i < downsampled_viewpoints->points.size(); i++) {

            pcl::FrustumCulling<pcl::PointNormal> fc;

            fc.setInputCloud(voxel_cloud);
            fc.setVerticalFOV(Vfov);
            fc.setHorizontalFOV(Hfov);
            fc.setNearPlaneDistance(near);
            fc.setFarPlaneDistance(far);

            /* #1: Get Points in frustum*/
            pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);

            pcl::PointNormal viewpoint = downsampled_viewpoints->points[i];
            Eigen::Vector3f position = Eigen::Vector3f( viewpoint.x, viewpoint.y, viewpoint.z);
            Eigen::Vector3f dir = Eigen::Vector3f(viewpoint.normal_x, viewpoint.normal_y, viewpoint.normal_z);
            Eigen::Matrix4f pose;
            // /* Find cospi and sinpi */
            float c1 = sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
            float s1 = dir[2];
            ///* Find cosO and sinO; if gimbal lock, choose (1,0) arbitrarily */
            float c2 = c1 ? dir[0] / c1 : 1.0;
            float s2 = c1 ? dir[1] / c1 : 0.0;
            //
            //return mat3(v.x, -s2, -s1 * c2,
            //    v.y, c2, -s1 * s2,
            //    v.z, 0, c1);
            pose << dir[0], -s2, -s1 * c2, position[0],
                dir[1], c2, -s1 * s2, position[1],
                dir[2], 0, c1, position[2],
                0.f, 0.f, 0.f, 1.f;

            //get frustum cull
            fc.setCameraPose(pose);
            fc.filter(*output);


            /* #2: Get Non occluded points*/
            output->sensor_origin_ = Eigen::Vector4f(position[0], position[1], position[2], 0);
            Eigen::Matrix3f rot = pose.block(0, 0, 3, 3);
            output->sensor_orientation_ = Eigen::Quaternionf( rot);
            pcl::VoxelGridOcclusionEstimationN voxelFilter;
            voxelFilter.setInputCloud(output);
            voxelFilter.setLeafSize(voxelRes, voxelRes, voxelRes);
            voxelFilter.initializeVoxelGrid();
            
            int state, ret;

            pcl::PointNormal pt, p1, p2;
            pcl::PointNormal point;
            std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
            std::vector<pcl::PointNormal> lineSegments;
            pcl::PointNormal linePoint;

            // iterate over the entire frustum points
            int count = 0;
            for (int i = 0; i < (int)output->points.size(); i++)
            {
                pcl::PointNormal ptest = output->points[i];
                Eigen::Vector3i ijk = voxelFilter.getGridCoordinates(ptest.x, ptest.y, ptest.z);
                // process all free voxels
                int index = voxelFilter.getCentroidIndexAt(ijk);
                Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate(ijk);
                point = pcl::PointNormal(centroid[0], centroid[1], centroid[2]);

                //if point in the voxel grid
                if (index != -1)
                {
                    out_ray.clear();
                    ret = voxelFilter.occlusionEstimation(state, out_ray, ijk);
                    //std::cout<<"State is:"<<state<<"\n";
                    //if state is not occupied
                    if (state != 1)
                    {
                        // estimate direction to target voxel
                        Eigen::Vector4f direction = centroid - output->sensor_origin_;
                        direction.normalize();
                        // estimate entry point into the voxel grid
                        float tmin = voxelFilter.rayBoxIntersection(output->sensor_origin_, direction);

                        //make line sedgment from sensor to target
                        if (tmin != -1)
                        {
                            count++;

                        }
                    }
                }

            }


            nb_coverage.push_back(count);
        }
        return  nb_coverage;
    }
   
}
