#pragma once

////print colors of verticies
//SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> chiki = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color >("v:color").first;
//for (SurfaceMesh::Vertex_index vi : surface.vertices())
//{
//    cout << "Red: " << (int)chiki[vi][0] << ",  Green: " << (int)chiki[vi][1] << ",  Blue: " << (int)chiki[vi][2] << ",  Alpha: " << (int)chiki[vi][3] << endl;
//    
//}


   // SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> new_colors;
   // bool created;
   // boost::tie(new_colors, created) = surface.add_property_map<SurfaceMesh::Vertex_index, CGAL::Color>("v:name", CGAL::Color::Color());
   // assert(created);


    //iterate over all faces and get their vetecies
    //int i = 0;
    //BOOST_FOREACH(boost::graph_traits<SurfaceMesh>::face_descriptor fd, faces(surface)) {
    //cout << "Face " << i << ": ";
    //for (SurfaceMesh::Vertex_index vd : vertices_around_face(surface.halfedge(fd), surface)) {
    //    std::cout << vd.idx() << "   ";
    //}
    //cout << endl;
    //i += 1;
    //}


//find center of mesh
//Point find_centre_of_mesh()
//{
//
//    const Tr& tr = triangulation();
//    double n_vertex = tr.number_of_vertices();
//
//    double mean_vtx[3] = { 0 };
//    Point p;
//    map<Vertex_handle, int> V;
//    int inum = 1;
//
//    for (Finite_vertices_iterator vit = tr.finite_vertices_begin(); vit != tr.finite_vertices_end(); ++vit)
//    {
//        p = Point(vit->point());
//        mean_vtx[0] += CGAL::to_double(p.x());
//        mean_vtx[1] += CGAL::to_double(p.y());
//        mean_vtx[2] += CGAL::to_double(p.z());
//    }
//
//    Point centre_point(mean_vtx[0] / n_vertex, mean_vtx[1] / n_vertex, mean_vtx[2] / n_vertex);
//
//    cout << "x: " << centre_point.x() << endl
//        << "y: " << centre_point.y() << endl
//        << "z: " << centre_point.z() << endl;
//
//    return centre_point;
//}


///**
// * Find the closest element in the surface to a given point in space,
// * The found point will be in the specified domain (tissue type)
// * */
//Point find_closest_element(Point target_p, int target_domain)
//{
//
//    double min_dist = DBL_MAX;
//    double this_dist;
//    Point centre_of_closest(0, 0, 0);
//    vector<Point> facet_points;
//
//    // Iterate over all facets
//    for (Facet_iterator facet_iterator = facets_in_complex_begin();
//        facet_iterator != facets_in_complex_end(); ++facet_iterator)
//    {
//
//        // Get domain number/tissue type using facet map
//        int this_tissue = facet_iterator->first->subdomain_index();
//
//        // Check facet is the tissue type we want
//        if (this_tissue == target_domain)
//        {
//            for (int i = 0; i < 4; ++i)
//            {
//
//                // We only want to store the three verticies of the cell that correspond to this facet
//                // ->second gives the index of the vertex opposite to the facet, which is the one we don't want
//                if (i != facet_iterator->second)
//                {
//                    facet_points.push_back(Point(facet_iterator->first->vertex(i)->point()));
//                }
//            }
//
//            //Calculate centre of the facet
//            Point centre_of_facet = CGAL::centroid(facet_points[0], facet_points[1], facet_points[2]);
//            //cout << "Centre: " << centre_of_facet <<endl;
//
//            // how far is this facet from the target point
//            this_dist = CGAL::squared_distance(target_p, centre_of_facet);
//
//            // If closer than current minimum, store it and update min distance
//            if (this_dist < min_dist)
//            {
//                min_dist = this_dist;
//                centre_of_closest = centre_of_facet;
//            }
//
//            // Empty facet_points, otherwise the [0] [1] [2] indexing used for CGAL::centroid won't do anything with new data
//            facet_points.clear();
//        }
//    }
//
//    cout << "Closest point to: " << target_p << " is: " << centre_of_closest << endl;
//    return centre_of_closest;
//}




//**
//* Iterate through all verticiesand find the min / max values in each dimensions.
//* These are the start / end bounds of the mesh.
//* */
//void C3t3_ind_mesh_bounds()
//{
//
//    const Tr& tr = triangulation();
//    double n_vertex = tr.number_of_vertices();
//    double x, y, z;
//
//    // Iterate through all vertices
//    for (Finite_vertices_iterator vit = tr.finite_vertices_begin(); vit != tr.finite_vertices_end(); ++vit)
//    {
//
//        auto current_vertex = vit->point();
//        x = CGAL::to_double(current_vertex.x());
//        y = CGAL::to_double(current_vertex.y());
//        z = CGAL::to_double(current_vertex.z());
//
//        // Check if current value is the bigger/smaller than current and update if so
//        if (x > x_max)
//            x_max = x;
//        if (x < x_min)
//            x_min = x;
//
//        if (y > y_max)
//            y_max = y;
//        if (y < y_min)
//            y_min = y;
//
//        if (z > z_max)
//            z_max = z;
//        if (z < z_min)
//            z_min = z;
//    }
//
//    x_mid = (x_min + x_max) / 2;
//    y_mid = (y_min + y_max) / 2;
//    z_mid = (z_min + z_max) / 2;
//}





//
//
//    //create PCL point cloud from surface mesh
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
////convert to pointcloud
//pcl::PointCloud< pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>());
//
////create point cloud
//cloud->width = surface.number_of_vertices();
//cloud->height = 1;
//cloud->is_dense = false;
//cloud->points.resize(surface.number_of_vertices());
//int i = 0;
/////
///// use IndicesPtr indices_ = shared_ptr<Indices> = shared_ptr<std::vector<index_t>>;
///// populate it
///// assign it via   sor.setindecies.
//for (SurfaceMesh::Vertex_index vi : surface.vertices())
//{
//    cloud->push_back(pcl::PointXYZ(surface.point(vi).x(), surface.point(vi).y(), surface.point(vi).z()));
//    i++;
//}
//
//std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
//
//pcl::VoxelGrid<pcl::PointXYZ>* sor = new pcl::VoxelGrid<pcl::PointXYZ>();
//// Create the filtering object
//sor->setInputCloud(cloud);
//sor->setLeafSize(0.1f, 0.1f, 0.1f);
//sor->filter(*filtered_cloud);
////
//std::cerr << "PointCloud after filtering: " << filtered_cloud->width * filtered_cloud->height
//<< " data points (" << pcl::getFieldsList(*filtered_cloud) << ")." << std::endl;
//
//





//selected_viewpoint: list of indices of selected viewpoints
//std::pair<int, pcl::PointCloud<pcl::PointXYZ>> knapsack(int index, std::vector<int> selected_viewpoint, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& viewpoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//
//    std::pair<int, pcl::PointCloud<pcl::PointXYZ>> selected_intersections = calc_nb_intersection(selected_viewpoint, viewpoints);
//    int total_nb_intersection = selected_intersections.first;
//    pcl::PointCloud<pcl::PointXYZ> vertices_observed = selected_intersections.second;
//
//    //base case: stop if surface covered
//    if (vertices_observed.size() >= cloud->points.size()) {
//        return selected_intersections;
//    }
//
//    //base case: no viewpoints lefdt
//    if (index >= viewpoints.size()) {
//        return selected_intersections;
//    }
//
//    std::pair<int, pcl::PointCloud<pcl::PointXYZ>> branch1 = knapsack(index + 1, selected_viewpoint, viewpoints, cloud);
//
//
//    std::vector<int> new_vec(selected_viewpoint);
//    new_vec.push_back(index);
//    std::pair<int, pcl::PointCloud<pcl::PointXYZ>> branch2 = knapsack(index + 1, selected_viewpoint, viewpoints, cloud);
//
//    //compare branch and coose the one with least intersection
//    if (branch1.first < branch2.first)return branch1;
//    return branch2;
//
//}




//std::vector<int> result;
//std::pair<int, pcl::PointCloud<pcl::PointXYZ>> x = knapsack(0, result, final_viewpoints, cloud);
//
//cout << "Number of Intersections: " << x.first << endl;
//cout << "Number of V covered: " << x.second.points.size() << endl;


   ///----------------------------------
   // std::vector<std::vector<int>> memo;
   // for (size_t i = 0; i < final_viewpoints.size()+1; i++)
   // {   
   //     std::vector<int> row;
   //     for (size_t j = 0; j < output_viewpoints_cloud.size()+1 ; j++)
   //     {
   //         row.push_back(0);
   //     }
   // }
   //
   // for (size_t i = 0; i < final_viewpoints.size(); i++)
   // {   
   //     for (size_t w = 0; w < output_viewpoints_cloud.size(); w++)
   //     {
   //
   //         // if the number of vertices observed <= w
   //                 // memo[i][w] = max ( final_viewpoints[i - 1].second.size() + K[i - 1][w - wt[i - 1]], K[i - 1][w]);
   //         i
   //         
   //
   //     }
   // }
   //
   //
   // std::pair<int, pcl::PointCloud<pcl::PointXYZ>> selected_intersections = calc_nb_intersection(selected_viewpoint, viewpoints);
   // int total_nb_intersection = selected_intersections.first;
   // pcl::PointCloud<pcl::PointXYZ> vertices_observed = selected_intersections.second;



    //std::vector<int> selected_viewpoints{ 0,2};
    //cout << "Position of index 0  "<< final_viewpoints[0].first.block(0, 3, 3, 1)<<endl;
    //cout << "Position of index 2  " << final_viewpoints[2].first.block(0, 3, 3, 1) << endl;
    //
    //SurfaceMesh surface1 = SurfaceMesh(surface);
    //SurfaceMesh surface2 = SurfaceMesh(surface);
    //SurfaceMesh surface4 = SurfaceMesh(surface);
    //MyMesh::color_visible_surface(final_viewpoints[0].second, surface1);
    //write_PLY(argv[3], surface1);
    //MyMesh::color_visible_surface(final_viewpoints[2].second, surface2);
    //write_PLY(argv[4], surface2); 
    //std::pair<int, pcl::PointCloud<pcl::PointXYZ>> result = calc_nb_intersection(selected_viewpoints, final_viewpoints);
    //cout << "NB intersections  " << result.first << endl;
    ////color union
    //MyMesh::color_visible_surface(result.second, surface4);
    //write_PLY(argv[5], surface4);


                                                         //generate power set
                                                                    // keep only sets that cover all vertices - calculate union when making set
                                                                    //pick one with smallest intersection value - calculate intersection sum   -- while going down the recursion add nb of intersectiona s number

                                                                //sort sets by nb of intersection, go from small to big and stop when all surface is covered


//std::pair<int, pcl::PointCloud<pcl::PointXYZ>> calc_nb_intersection(const std::vector<int>& selected_viewpoints, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& viewpoints) {
//    //no intersection if there is 1 or 0 elements --> 0 intersections
//    if (selected_viewpoints.size() <= 1) return std::pair<int, pcl::PointCloud<pcl::PointXYZ>>(0, pcl::PointCloud<pcl::PointXYZ>());
//
//    int nb_inter = 0;
//
//    pcl::PointCloud<pcl::PointXYZ> curr = viewpoints[selected_viewpoints[0]].second;
//
//    for (unsigned int i = 1; i < selected_viewpoints.size(); i++)
//    {
//
//        std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>> viewpoint = viewpoints[selected_viewpoints[i]];
//        pcl::PointCloud<pcl::PointXYZ> set = viewpoint.second;
//
//        //make union and calculate nb of intersection while at it
//        for (size_t set_idx = 0; set_idx < set.size(); set_idx++)
//        {
//
//            pcl::PointXYZ set_point = set[set_idx];
//            bool found = false;
//            for (size_t curr_idx = 0; curr_idx < curr.size(); curr_idx++)
//            {
//
//                pcl::PointXYZ curr_point = curr[curr_idx];
//                //if same point ==> intersection
//                if (std::abs(curr_point.x - set_point.x) < 0.00001f && std::abs(curr_point.y - set_point.y) < 0.00001 && std::abs(curr_point.z - set_point.z) < 0.00001) {
//                    nb_inter++;
//                    found = true;
//                    break;
//                }
//
//            }
//
//            if (!found) {
//                curr.push_back(set_point);
//            }
//        }
//
//    }
//
//    return std::pair<int, pcl::PointCloud<pcl::PointXYZ>>(nb_inter, curr);
//}