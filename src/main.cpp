#include "MyMesh.h"
#include "IO.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "VoxelGridOcclusionEstimationT.h"

int main(int argc, char* argv[])
{
    Polyhedron m;
    SurfaceMesh surface;
    import_OFF_file(m, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (m.is_empty() && surface.is_empty()) return 1;

    MyMesh::print_mesh_info(m);

    //write_PLY(argv[2], surface);
    //write_OFF(argv[2], surface);

    //load veticies and colkor dtaa to vectors
    //std::vector<Point> verts;
    //std::vector<CGAL::Color> cols;
    //MyMesh::load_verticies_and_color(surface, verts, cols);

    //MyMesh::color_surface(surface, 0, 255, 0, 255);
    //MyMesh::segment_mesh(surface);
    //std::cout << "Number of Segments: " << MyMesh::segments_vec.size() << std::endl;

    /////////////////////----------------------------------------------------------------------------------------
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCopy = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    
    //create point cloud
    cloud->width = surface.number_of_vertices();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(surface.number_of_vertices());
    int i = 0;

    /// use IndicesPtr indices_ = shared_ptr<Indices> = shared_ptr<std::vector<index_t>>;
    /// populate it
    /// assign it via   sor.setindecies.
    for (SurfaceMesh::Vertex_index vi : surface.vertices())
    {
        cloud->push_back(pcl::PointXYZ(surface.point(vi).x(), surface.point(vi).y(), surface.point(vi).z()));
        i++;
    }

    cloudCopy->points = cloud->points;
    float voxelRes, OriginalVoxelsSize, viewEntropy;
    double id;
    pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
    Eigen::Vector3i  max_b1, min_b1;

    double maxAccuracyError, minAccuracyError;
    bool AccuracyMaxSet;
    std::string frame_id;
    voxelRes = 0.1f;
    OriginalVoxelsSize = 0.0;
    id = 0.0;
    viewEntropy = 0.0;
    voxelFilterOriginal.setInputCloud(cloud);
    voxelFilterOriginal.setLeafSize(voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates();
    std::cout << "min_b1 :" << min_b1<< "\n";
    std::cout << "max_b1 :" << max_b1 << "\n";

    //calculate voxel size
    for (int kk = min_b1.z(); kk <= max_b1.z(); ++kk)
    {
        for (int jj = min_b1.y(); jj <= max_b1.y(); ++jj)
        {
            for (int ii = min_b1.x(); ii <= max_b1.x(); ++ii)
            {
                Eigen::Vector3i ijk1(ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt(ijk1);
                if (index1 != -1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }

    cout << "Voxel Grid Size: "<< OriginalVoxelsSize << endl;
    std::cerr << "PointCloud after filtering: " << voxelFilterOriginal.getFilteredPointCloud().width * voxelFilterOriginal.getFilteredPointCloud().height
        << " data points (" << pcl::getFieldsList(voxelFilterOriginal.getFilteredPointCloud()) << ")." << std::endl;
    return EXIT_SUCCESS;
}