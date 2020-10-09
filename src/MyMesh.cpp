#include "MyMesh.h"

namespace MyMesh {
    void print_mesh_info(Polyhedron& m) {
        std::string meshType = "";

        if (m.is_pure_triangle()) {
            meshType = std::string("Triangle");
        }
        else if (m.is_pure_quad()) {
            meshType = std::string("Quad");
        }
        else {
            meshType = std::string("General");
        }

        //loop over verticies
        double valenceSum = 0;
        int minValence = -1;
        int maxValence = -1;
        //for each vertex
        for (Polyhedron::Vertex_const_iterator vertexIter = m.vertices_begin(); vertexIter != m.vertices_end(); ++vertexIter) {

            //get valence
            int valence = vertexIter->degree();

            //update min and max
            if (minValence < 0 || valence < minValence) minValence = valence;
            if (maxValence < 0 || valence > maxValence) maxValence = valence;

            valenceSum += valence;
        }

        double meanValence = valenceSum / m.size_of_vertices();

        //check nonplanar faces
        int numNonpalanarFaces = 0;
        double degreeSum = 0;
        int minDegree = -1;
        int maxDegree = -1;

        for (Polyhedron::Face_const_iterator faceIter = m.facets_begin(); faceIter != m.facets_end(); ++faceIter) {

            //get degree of face
            int degree = faceIter->facet_degree();
            //face can be only nonplar if degree more than three
            if (degree >= 4) {

                //use circulator to go through all half facets
                Polyhedron::Facet::Halfedge_around_facet_const_circulator halfEdgeCir = faceIter->facet_begin();

                //get first vertex of face
                const Point& v0 = halfEdgeCir->vertex()->point();
                ++halfEdgeCir;

                //get second vertex of face
                const Point& v1 = halfEdgeCir->vertex()->point();
                ++halfEdgeCir;

                //get third vertex of face
                const Point& v2 = halfEdgeCir->vertex()->point();
                ++halfEdgeCir;

                //check if each vertex is coplanar with first three verticies
                for (int i = 3; i < degree; ++i) {

                    //get next vertex of face
                    const Point& v = halfEdgeCir->vertex()->point();
                    ++halfEdgeCir;

                    if (!CGAL::coplanar(v0, v1, v2, v)) {
                        numNonpalanarFaces++;

                        std::cout << "nonplanar face detected: \n";
                        std::cout << "(" << v0.x() << ", " << v0.y() << ", " << v0.z() << ")\n";
                        std::cout << "(" << v1.x() << ", " << v1.y() << ", " << v1.z() << ")\n";
                        std::cout << "(" << v2.x() << ", " << v2.y() << ", " << v2.z() << ")\n";
                        std::cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")\n";
                        break;
                    }
                }
            }

            //update min degree
            if (minDegree < 0 || degree < minDegree) minDegree = degree;
            //update max degree
            if (maxDegree < 0 || degree > maxDegree) maxDegree = degree;

            degreeSum += degree;
        }

        double meanDegree = degreeSum / m.size_of_facets();

        //normalize border  to use sizeofborderedges
        m.normalize_border();

        //info
        std::cout
            << "Mesh Type: " << meshType << "\n"
            << "Number Vertices: " << m.size_of_vertices() << "\n"
            << "Number Edges: " << m.size_of_halfedges() / 2 << "\n"
            << "Number of Border Edges: " << m.size_of_border_edges() << "\n"
            << "Number of facets: " << m.size_of_facets() << "\n"
            << "Number of halfEdges: " << m.size_of_halfedges() << "\n"
            << "Mean vertex Valence: " << meanValence << "\n"
            << "Min vertex Valence: " << minValence << "\n"
            << "Max vertex Valence: " << maxValence << "\n"
            << "Mean Facet Degree: " << meanDegree << "\n"
            << "Min Facet Degree: " << minDegree << "\n"
            << "Max Facet Degree: " << maxDegree << "\n"
            << "Number Nonplanar Facets: " << numNonpalanarFaces << "\n";

    }


    void load_verticies_and_color(SurfaceMesh surface, std::vector<Point>& verts, std::vector<CGAL::Color>& cols) {
        SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> vcolors = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color >("v:color").first;
        bool colorExists = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color>("v:color").second;

        if (!colorExists) {
            std::cerr << "COLOR DOES NOT EXIST";
            for (SurfaceMesh::Vertex_index vi : surface.vertices())
            {
                verts.push_back(surface.point(vi));
            }
        }
        else {
            for (SurfaceMesh::Vertex_index vi : surface.vertices())
            {
                cols.push_back(vcolors[vi]);
                verts.push_back(surface.point(vi));
            }
        }
    }


    void color_surface(SurfaceMesh& surface, int r, int g, int b, int a) {
        //update colors
        SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> vcolors = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color >("v:color").first;
        bool colorExists = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color>("v:color").second;


        if (!colorExists) {
            std::cerr << "COLOR DOES NOT EXIST adding default color";
        }
        else {
            for (SurfaceMesh::Vertex_index vi : surface.vertices())
            {
                vcolors[vi].set_rgb(r, g, b, a);
            }
        }
    }

    void MyMesh::color_visible_surface(pcl::PointCloud<pcl::PointXYZ> visible_s, SurfaceMesh& surface) {
        SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> vcolors = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color >("v:color").first;
        for (int i = 0; i < visible_s.points.size(); i++) {
            pcl::PointXYZ p = visible_s.points[i];
            for (SurfaceMesh::Vertex_index vi : surface.vertices())
            {
                if (std::abs(p.x - (float)surface.point(vi).x()) <= 0.00001f && std::abs(p.y - (float)surface.point(vi).y()) <= 0.00001f && std::abs(p.z - (float)surface.point(vi).z()) <= 0.00001f) {
                    vcolors[vi].set_rgb(255, 0, 0, 255);
                }
            }
        }
    }


    void segment_mesh(SurfaceMesh surface) {

        std::vector<SurfaceMesh*> segments_vec;
        typedef SurfaceMesh::Property_map<face_descriptor, double> Facet_double_map;
        Facet_double_map sdf_property_map;
        sdf_property_map = surface.add_property_map<face_descriptor, double>("f:sdf").first;
        // compute SDF values
            // We can't use default parameters for number of rays, and cone angle
            // and the postprocessing
        CGAL::sdf_values(surface, sdf_property_map, 2.0 / 3.0 * CGAL_PI, 10, true);
        // create a property-map for segment-ids
        typedef SurfaceMesh::Property_map<face_descriptor, std::size_t> Facet_int_map;
        Facet_int_map segment_property_map = surface.add_property_map<face_descriptor, std::size_t>("f:sid").first;;
        const std::size_t number_of_clusters = 4;       // use 4 clusters in soft clustering
        const double smoothing_lambda = 0.4;  // importance of surface features, suggested to be in-between [0,1]
        // Note that we can use the same SDF values (sdf_property_map) over and over again for segmentation.
        // This feature is relevant for segmenting the mesh several times with different parameters.
        size_t number_of_segments = CGAL::segmentation_from_sdf_values(surface, sdf_property_map, segment_property_map, number_of_clusters, smoothing_lambda);

        std::cout << "Number of segments: " << number_of_segments << std::endl;

        typedef CGAL::Face_filtered_graph<SurfaceMesh> Filtered_graph;
        //print area of each segment and then put it in a Mesh and print it in an OFF file
        Filtered_graph segment_mesh(surface, 0, segment_property_map);
        for (std::size_t id = 0; id < number_of_segments; ++id)
        {
            if (id > 0)
                segment_mesh.set_selected_faces(id, segment_property_map);
            //std::cout << "Segment " << id << "'s area is : " << CGAL::Polygon_mesh_processing::area(segment_mesh) << std::endl;
            SurfaceMesh* out = new SurfaceMesh();

            //doesn't do colors -----------------------
            CGAL::copy_face_graph(segment_mesh, *out);
            // create a color property map
            SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color > c;
            bool created;
            boost::tie(c, created) = out->add_property_map<vertex_descriptor, CGAL::Color>("v:color", CGAL::Color::Color());
            assert(created);
            color_surface(*out, 0, 250, 0, 250);


            //
            //segments_vec.push_back(out);

            std::string out_s = "out/Segment_" + std::to_string(id) + ".ply";
            IO::write_PLY(out_s, *out);
        }
    }


    void convert_to_pointcloud(SurfaceMesh& surface, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

        //create point cloud
        //cloud->width = surface.number_of_vertices();
        //cloud->height = 1;
        //cloud->is_dense = false;
        //cloud->points.resize(surface.number_of_vertices());
        int i = 0;
        /// use IndicesPtr indices_ = shared_ptr<Indices> = shared_ptr<std::vector<index_t>>;
        /// populate it
        /// assign it via   sor.setindecies.
        for (SurfaceMesh::Vertex_index vi : surface.vertices())
        {
            cloud->push_back(pcl::PointXYZ(surface.point(vi).x(), surface.point(vi).y(), surface.point(vi).z()));
            i++;
        }
    }


    double max_coordinate(const Polyhedron& poly)
    {
        double max_coord = -std::numeric_limits<double>::infinity();
        for (Polyhedron::Vertex_handle v : vertices(poly))
        {
            Point p = v->point();
            max_coord = (std::max)(max_coord, p.x());
            max_coord = (std::max)(max_coord, p.y());
            max_coord = (std::max)(max_coord, p.z());
        }
        return max_coord;
    }


    bool point_inside_mesh(CGAL::Side_of_triangle_mesh<Polyhedron, Kernel>& inside, Eigen::Vector4f point_eigen) {
        Kernel::Point_3 point = Kernel::Point_3(point_eigen.x(), point_eigen.y(), point_eigen.z());
        CGAL::Bounded_side res = inside(point);
        if (res == CGAL::ON_BOUNDED_SIDE || res == CGAL::ON_BOUNDARY) return true;
        return false;
    }



    void remove_points_inside_mesh(Polyhedron poly, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& viewpoints, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& final_viewpoints, std::vector< Kernel::Point_3>& viewpoints_cloud) {
        //remove viewpoints that are inside of the mesh
        CGAL::Side_of_triangle_mesh<Polyhedron, Kernel> inside(poly);
        double size = max_coordinate(poly);
        int nb_inside = 0;
        int nb_boundary = 0;
        for (std::size_t i = 0; i < viewpoints.size(); i++)
        {
            Eigen::Vector3f point_eigen = viewpoints[i].first.block(0, 3, 3, 1);
            Kernel::Point_3 point = Kernel::Point_3(point_eigen.x(), point_eigen.y(), point_eigen.z());
            CGAL::Bounded_side res = inside(point);
            if (res == CGAL::ON_BOUNDED_SIDE) {
                ++nb_inside;

            }
            else if (res == CGAL::ON_BOUNDARY) { ++nb_boundary; }
            else {
                final_viewpoints.push_back(viewpoints[i]);
                viewpoints_cloud.push_back(point);
            }
        }

        //std::cerr << "  " << nb_inside << " points inside " << std::endl;
        //std::cerr << "  " << nb_boundary << " points on boundary " << std::endl;
        //std::cerr << "  " << viewpoints.size() - nb_inside - nb_boundary << " points outside " << std::endl;
        //

        //std::cerr << "TEST CLOUD SIZE " << viewpoints_cloud.size() << std::endl;
    }


}