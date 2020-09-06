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




//SEGMENTING MESH SUING CGAL
/// run through each mesh in INPUT_DIR (getting SDF values only once)
//	// 		run through k number of clusters
//	//			run through smoothing lambdas
//	//				segment and write .OFF file
//double tot_toc = 0;
//int curr_ix = 0;
//int n_pcls = meshes_filenames.size();
//for (std::vector<std::string>::iterator it = meshes_filenames.begin(); it != meshes_filenames.end(); ++it) {
//	// variables to measure time elapsed
//	curr_ix++;
//	std::clock_t tic = std::clock();
//
//	// get filepath to write to
//	std::string mesh_filename = *it;
//	std::string mesh_filepath = INPUT_DIR + mesh_filename;
//
//	// get mesh from .OFF file
//	// if there is a problem continue the loop for next mesh
//	Polyhedron mesh;
//	if (!get_mesh_from_off_file(mesh_filepath, mesh))
//		continue;
//
//	// assign id field for each face
//	if (VERBOSE) std::cout << "Assigning a different ID to each face...";
//	std::size_t facet_id = 0;
//	for (Polyhedron::Facet_iterator facet_it = mesh.facets_begin(); facet_it != mesh.facets_end(); ++facet_it, ++facet_id)
//		facet_it->id() = facet_id;
//	if (VERBOSE) std::cout << "OK" << std::endl;
//
//	// create a property-map for SDF values
//	// to access SDF values with constant-complexity
//	if (VERBOSE) std::cout << "Creating an index for SDF values to access them with constant-complexity...";
//	std::vector<double> sdf_values(mesh.size_of_facets());
//	Facet_with_id_pmap<double> sdf_property_map(sdf_values);
//	CGAL::sdf_values(mesh, sdf_property_map);
//	if (VERBOSE) std::cout << "OK" << std::endl;
//
//	// create a property-map for segment IDs
//	// so we can access a face's segment ID with constant-complexity
//	if (VERBOSE) std::cout << "Creating an index for face segment IDS to access them with constant-complexity...";
//	std::vector<std::size_t> segment_ids(mesh.size_of_facets());
//	Facet_with_id_pmap<std::size_t> segment_property_map(segment_ids);
//	if (VERBOSE) std::cout << "OK" << std::endl;
//	std::cout << std::endl;
//
//	// run through ks and lambdas, segment and write to .OFF file
//	double tot_toc_per_mesh = 0;
//	int curr_ix_mesh = 0;
//	int n_ks_tries = 0;
//	for (int k = K_START; k <= K_END; k += K_STEP)
//		n_ks_tries++;
//	int n_lambda_tries = 0;
//	for (float lambda = LAMBDA_START; lambda <= LAMBDA_END; lambda += LAMBDA_STEP)
//		n_lambda_tries++;
//	for (int k = K_START; k <= K_END; k += K_STEP) {
//		curr_ix_mesh++;
//		int curr_ix_lambda = 0;
//		double tot_toc_per_lambda = 0;
//		for (float lambda = LAMBDA_START; lambda <= LAMBDA_END; lambda += LAMBDA_STEP) {
//			curr_ix_lambda++;
//			// segment the mesh with params k and lambda
//			if (VERBOSE) std::cout << "Segmenting...";
//			int n_segms = CGAL::segmentation_from_sdf_values(mesh, sdf_property_map, segment_property_map, k, lambda);
//			if (VERBOSE) std::cout << "OK" << std::endl << "Number of segments: " << n_segms << std::endl;
//
//			// write mesh to .PLY or .OFF file
//			if (INPUT_FILE_EXT.compare("ply") == 0)
//				write_mesh_to_ply_file(OUTPUT_DIR + mesh_filename, mesh, segment_property_map, n_segms, k, lambda);
//			else
//				write_mesh_to_off_file(OUTPUT_DIR + mesh_filename, mesh, segment_property_map, n_segms, k, lambda);
//
//
//			if (VERBOSE) std::cout << std::endl;
//			tot_toc_per_lambda += double(std::clock() - tic) / CLOCKS_PER_SEC;
//			std::cout << std::endl;
//			DisplayEstimatedTimeOfLoop(tot_toc_per_lambda, curr_ix_lambda, n_lambda_tries, "Loop for each lambda per mesh ");
//		}
//		tot_toc_per_mesh += double(std::clock() - tic) / CLOCKS_PER_SEC;
//		std::cout << std::endl;
//		DisplayEstimatedTimeOfLoop(tot_toc_per_mesh, curr_ix_mesh, n_ks_tries, "Loop for each K per mesh ");
//		std::cout << std::endl;
//	}
//	tot_toc += double(std::clock() - tic) / CLOCKS_PER_SEC;
//	std::cout << std::endl;
//	DisplayEstimatedTimeOfLoop(tot_toc, curr_ix, n_pcls, "Loop for all meshes ");
//	std::cout << std::endl;
//}