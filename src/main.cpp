#include "MyMesh.h"
#include "IO.h"
#include <pcl/point_types.h>
#include "Occlusion_Culling.h"
#include <fstream>
#include <iostream>
#include <cmath> 
#include <ctime>
#include "PathPlanner.h"
#include <CGAL/Side_of_triangle_mesh.h>

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

int main(int argc, char* argv[])
{
    Polyhedron poly;
    SurfaceMesh surface;
    import_OFF_file(poly, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (poly.is_empty() && surface.is_empty()) return 1;
    
    //MyMesh::segment_mesh(surface);
    //return 1;
    //MyMesh::print_mesh_info(m);
    /////////////////////----------------------------------------------------------------------------------------
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>); 

    

    MyMesh::convert_to_pointcloud(surface, cloud);

    PathPlanner pp(cloud);
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> viewpoints =pp.extract_surfaces_routine();
    std::cout << "map size: " << viewpoints.size() << std::endl;
    
    
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> final_viewpoints;
    std::vector< Kernel::Point_3> viewpoints_cloud;

    //remove viewpoints that are inside of the mesh
    CGAL::Side_of_triangle_mesh<Polyhedron, Kernel> inside(poly);
    double size = max_coordinate(poly);
    int nb_inside = 0;
    int nb_boundary = 0;
    for (std::size_t i = 0; i < viewpoints.size(); ++i)
    {     
        Eigen::Vector3f point_eigen = viewpoints[i].first.block(0, 3, 3, 1);
        Kernel::Point_3 point = Kernel::Point_3(point_eigen.x(), point_eigen.y(), point_eigen.z()) ;
        CGAL::Bounded_side res = inside(point);
        if (res == CGAL::ON_BOUNDED_SIDE) {
            ++nb_inside;
            
        }else if (res == CGAL::ON_BOUNDARY) { ++nb_boundary; }
        else {
            final_viewpoints.push_back(viewpoints[i]);
            viewpoints_cloud.push_back(point);
        }
    }

    std::cerr << "  " << nb_inside << " points inside " << std::endl;
    std::cerr << "  " << nb_boundary << " points on boundary " << std::endl;
    std::cerr << "  " << viewpoints.size() - nb_inside - nb_boundary << " points outside " << std::endl;


    std::cerr << " TEST CLOUD SIZE " << viewpoints_cloud.size() << std::endl;
    write_PLY(argv[2], viewpoints_cloud);


   ///color visible surface
   // MyMesh::color_visible_surface(visible_s, surface);
   // write_PLY(argv[2], surface);



   
   return EXIT_SUCCESS;
}


