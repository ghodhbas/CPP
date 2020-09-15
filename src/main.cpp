#include "MyMesh.h"
#include "IO.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


int main(int argc, char* argv[])
{
    Polyhedron m;
    SurfaceMesh surface;
    import_OFF_file(m, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (m.is_empty() && surface.is_empty()) return 1;

    MyMesh::print_mesh_info(m);

    write_PLY(argv[2], surface);
    //write_OFF(argv[2], surface);


    //load veticies and colkor dtaa to vectors
    //std::vector<Point> verts;
    //std::vector<CGAL::Color> cols;
    //MyMesh::load_verticies_and_color(surface, verts, cols);

    //MyMesh::color_surface(surface, 0, 255, 0, 255);
    //MyMesh::segment_mesh(surface);

    //std::cout << "Number of Segments: " << MyMesh::segments_vec.size() << std::endl;
    /////////////////////----------------------------------------------------------------------------------------


    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //
    //for (unsigned int i = 0; i < 10; ++i)
    //{
    //    pcl::PointXYZ p; p.x = i; p.y = i; p.z = i;
    //    cloud->points.push_back(p);
    //}
    //
    //pcl::PointXYZ p; p.x = 5.1; p.y = 5.1; p.z = 5.1;
    //cloud->points.push_back(p);
    //
    //std::cout << "Number of points before: " << cloud->points.size() << std::endl;
    //
    //pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    //voxel_grid.setInputCloud(cloud);
    //voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    //voxel_grid.filter(*cloud);
    //
    //std::cout << "Number of points after: " << cloud->points.size() << std::endl;

    //create PCL point cloud from surface mesh

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //convert to pointcloud
    pcl::PointCloud< pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>());
    pcl::Filter<pcl::PointXYZ>::PointCloud::Ptr  ptr_cloud;


    //create point cloud
    cloud->width = surface.number_of_vertices();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(surface.number_of_vertices());
    int i = 0;
    ///
    /// use IndicesPtr indices_ = shared_ptr<Indices> = shared_ptr<std::vector<index_t>>;
    /// populate it
    /// assign it via   sor.setindecies.
    for (SurfaceMesh::Vertex_index vi : surface.vertices())
    {
        cloud->push_back(pcl::PointXYZ(surface.point(vi).x(), surface.point(vi).y(), surface.point(vi).z()));
        i++;
    }

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;



    pcl::VoxelGrid<pcl::PointXYZ>* sor = new pcl::VoxelGrid<pcl::PointXYZ>();
    // Create the filtering object
    sor->setInputCloud(cloud);
    sor->setLeafSize(0.1f, 0.1f, 0.1f);
    sor->filter(*filtered_cloud);
    //
    std::cerr << "PointCloud after filtering: " << filtered_cloud->width * filtered_cloud->height
        << " data points (" << pcl::getFieldsList(*filtered_cloud) << ")." << std::endl;

    return EXIT_SUCCESS;
}