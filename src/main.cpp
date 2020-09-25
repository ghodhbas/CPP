#include "MyMesh.h"
#include "IO.h"
#include <pcl/point_types.h>
#include "Occlusion_Culling.h"
#include <fstream>
#include <iostream>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <cmath> 


int main(int argc, char* argv[])
{
    Polyhedron m;
    SurfaceMesh surface;
    import_OFF_file(m, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (m.is_empty() && surface.is_empty()) return 1;

    //MyMesh::print_mesh_info(m);

    //load veticies and colkor dtaa to vectors
    //std::vector<Point> verts;
    //std::vector<CGAL::Color> cols;
    //MyMesh::load_verticies_and_color(surface, verts, cols);

    //MyMesh::color_surface(surface, 0, 255, 0, 255);
    //MyMesh::segment_mesh(surface);
    //std::cout << "Number of Segments: " << MyMesh::segments_vec.size() << std::endl;

    /////////////////////----------------------------------------------------------------------------------------
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>); 
    
    MyMesh::convert_to_pointcloud(surface, cloud);

    OcclusionCulling* OC = new OcclusionCulling(cloud);
    UAV* drone = new UAV();
   
    pcl::PointCloud<pcl::PointXYZ> visible_s = OC->extractVisibleSurface(*drone);
    SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> vcolors = surface.property_map<SurfaceMesh::Vertex_index, CGAL::Color >("v:color").first;
    for (int i = 0; i < visible_s.points.size(); i++) {
        pcl::PointXYZ p = visible_s.points[i];
        for (SurfaceMesh::Vertex_index vi : surface.vertices())
        {   
            if ( std::abs( p.x - (float)surface.point(vi).x())<= 0.00001f   && std::abs(p.y - (float)surface.point(vi).y()) <= 0.00001f && std::abs(p.z - (float)surface.point(vi).z()) <= 0.00001f) {
                vcolors[vi].set_rgb(255, 0, 0, 255);
            }
        }
    }
    write_PLY(argv[2], surface);

   return EXIT_SUCCESS;
}