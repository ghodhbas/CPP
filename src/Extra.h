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