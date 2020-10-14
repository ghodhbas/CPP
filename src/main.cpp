#include <pcl/point_types.h>
#include "Occlusion_Culling.h"
#include <fstream>
#include <iostream>
#include <cmath> 
#include <ctime>
#include "PathPlanner.h"
#include "IO.h"


int main(int argc, char* argv[])
{
    Polyhedron poly;
    SurfaceMesh surface;



    //set up MEesh
    IO::import_OFF_file(poly, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (poly.is_empty() && surface.is_empty()) return 1;
    //MyMesh::print_mesh_info(m);

    //make point cloud from vertices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    MyMesh::convert_to_pointcloud(surface, cloud);
    cout << "Number of points in the Point cloud: " << cloud->points.size() << endl<<endl;
    
    //MyMesh::segment_mesh(surface);

    /*--------------------------------             START FINDING PATH ALGORITHM    ---------------------          */
    cout << "Path Planning Intialization..." << endl;
    PathPlanner pp(cloud);
    cout << "Path Planning Intialization Complete" <<endl<< endl;

    //generate all the viewpoints where camera can be and the vertices they see
    cout << "Extracting surfaces from free voxels as viewpoints..." << endl;
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> viewpoints =pp.extract_surfaces_routine();
    cout << "Extraction complete" << endl;
    std::cout << "Total Number of Viewpoints: " << viewpoints.size() << std::endl<<endl;
    
    //remove all viewpoints that are inside of the mesh 
    cout << "Removing invalid viewpoints..." << endl;
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> final_viewpoints;
    std::vector< Kernel::Point_3> viewpoints_cloud;
    MyMesh::remove_points_inside_mesh(poly, viewpoints, final_viewpoints, viewpoints_cloud);
    cout << "Removing invalid viewpoints complete" << endl;
    cout << "NB viewpoints after cleaning: " << final_viewpoints.size() << endl<<endl;


    //Solve the set cover problem to get the lowest number of viewpoints that cover the surface  ---   Maximaze viewpoint-mesh intersection, remove intersectiona nd viewpoint, repeat
    cout << "Applying Set Cover: Generating Solution Viewpoints" << endl;
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>> Solution = pp.greedy_set_cover(cloud, final_viewpoints);
    cout << "Number of viewpoints to cover the paths " << Solution.size() <<endl<< endl;


    //GEnerate output (colored surface + viewpoints)
    //std::vector< Kernel::Point_3> output_viewpoints_cloud;
    //for (size_t i = 0; i < Solution.size(); i++)
    //{
    //    Eigen::Matrix4f pose = Solution[i].first;
    //    Eigen::Vector3f coord = pose.block(0, 3, 3, 1);
    //    output_viewpoints_cloud.push_back(Kernel::Point_3(coord.x(), coord.y(), coord.z()));
    //    MyMesh::color_visible_surface(Solution[i].second, surface);
    //}
    ////output mesh
    //IO::write_PLY(argv[3], surface);
    ////output point cloud for viewpoints
    //IO::write_PLY(argv[2], output_viewpoints_cloud);


    //saves the indices withion the graph where the viewpoints are located
    cout << "Graph Construction from Voxel Grid..." << endl;
    std::vector<int> viewpoint_graph_idx;
    pp.construct_graph(poly, Solution, viewpoint_graph_idx);
    cout << "Graph Construction Complete" << endl<< endl;
    //construct all the pairs from the viewpoints and calculate the distances between them -- edge list
    cout << "Calculating shortest distances between pairs of viewpoints..." << endl;
    vector<std::pair<int, int>> pair_vec;
    std::map<int, std::map<int, int>> distance_map  =  pp.calculate_distances(viewpoint_graph_idx, pair_vec);
    cout << "Distances calculation Complete" << endl << endl;

    cout << "Constructing Minimun Spanning tree between solution viewpoints..." << endl;
    Edge_Graph MST = pp.construct_MST(pair_vec, distance_map);
    cout << "MST CONSTRUCTED" << endl<<endl;


   ///color visible surface
   // MyMesh::color_visible_surface(visible_s, surface);
   // write_PLY(argv[2], surface);

   return EXIT_SUCCESS;
}


