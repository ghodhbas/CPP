#include <pcl/point_types.h>
#include "Occlusion_Culling.h"
#include <fstream>
#include <iostream>
#include <cmath> 
#include <ctime>
#include "PathPlanner.h"
#include "IO.h"
#include "LayeredPP.h"

/* METHOD 1: Set cover + TSP  */
void method_1(Polyhedron& poly, SurfaceMesh& surface, char* argv[]) {
    //make point cloud from vertices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    MyMesh::convert_to_pointcloud(surface, cloud);
    cout << "Number of points in the Point cloud: " << cloud->points.size() << endl << endl;

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

    vector<Eigen::Vector3f > path = pp.generate_path(MST);
    IO::write_PLY(argv[4], path);


   ///color visible surface
   // MyMesh::color_visible_surface(visible_s, surface);
   // write_PLY(argv[2], surface);
}

int main(int argc, char* argv[])
{
    Polyhedron poly;
    SurfaceMesh surface;

    //set up MEesh
    IO::import_OFF_file(poly, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (poly.is_empty() && surface.is_empty()) return 1;
    //MyMesh::print_mesh_info(m);
    
    //MyMesh::segment_mesh(surface);

    /*--------------------------------  METHOD 1: Set cover + TSP --------           START FINDING PATH ALGORITHM    ---------------------          */
    //method_1(poly, surface,  argv)
   

    /*--------------------------------  METHOD 2: layer +normal construction & TSP --------           START FINDING PATH ALGORITHM    ---------------------          */
    LayeredPP lpp( 0.3f, 3.f);

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));

    
    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);

    //cout << "MIN POINT" << min_max.first << endl << endl;
    //cout << "Max POINT" << min_max.second << endl<<  endl;

    //Step 3: voxalize the whole cloud and get cloud
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 1.f);
    //generate viewpoints from mesh grid and make them into a filtered point cloud
    typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
    Viewpoints viewpoints = lpp.generate_viewpoints(mesh_voxelgrid);
    pcl::PointCloud<pcl::PointNormal>::Ptr viewpoints_cloud= pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    for (size_t i = 0; i < viewpoints.size(); i++)
    {   
        
        pcl::PointNormal point = pcl::PointNormal(viewpoints.at(i).first[0], viewpoints.at(i).first[1], viewpoints.at(i).first[2], viewpoints.at(i).second[0], viewpoints.at(i).second[1], viewpoints.at(i).second[2]);
        viewpoints_cloud->push_back(point);
    }
    //calculate BBox of cloud
    std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max = MyMesh::get_cloud_BBOX(viewpoints_cloud);



    //Step 4: split the  viewpoints into layers
    int nb_layers = 6;
    vector< pcl::PointCloud<pcl::PointNormal>::Ptr> layer_viewpoints = lpp.construct_layers(viewpoints_cloud, nb_layers, min_max);



    //Step 5: Voxelize every layer of viewpoints and make input for TSP
    vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> viewpoints_voxel_layers = lpp.voxelize_layers(layer_viewpoints, 3.f);
    vector<Viewpoints> downsampled_viewpoints_perlayer;
    for (int i = 0; i < viewpoints_voxel_layers.size(); i++) {
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered_layer_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr( new pcl::PointCloud <pcl::PointNormal>(viewpoints_voxel_layers[i].getFilteredPointCloud()));
        //pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> voxel_layer = viewpoints_voxel_layers[i];
        Viewpoints layer;
        for (size_t j = 0; j < filtered_layer_cloud->points.size(); j++)
        {
            pcl::PointNormal point = filtered_layer_cloud->at(j);
            //std::pair<Eigen::Vector3f, Eigen::Vector3f> tmp = std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z));
            layer.push_back(std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z)));
        }
        //layer_viewpoints.push_back(layer);
        downsampled_viewpoints_perlayer.push_back(layer);
    }


    // step 7: solve TSP on every layer
    vector< vector<Eigen::Vector3f >> paths_list;
    for (size_t i = 0; i < downsampled_viewpoints_perlayer.size(); i++)
    {
        //construct list of viewpoints  for that layer
        Viewpoints layer = downsampled_viewpoints_perlayer[i];
        vector<std::pair<int, int>> pair_vec;
        std::map<int, std::map<int, float>> distance_map = lpp.calculate_distances(layer, pair_vec );
        //cout << "Constructing Minimun Spanning tree between solution viewpoints..." << endl;
        Edge_Graph MST = lpp.construct_MST(pair_vec, distance_map);
        //cout << "MST CONSTRUCTED" << endl << endl;
        vector<Eigen::Vector3f > path = lpp.generate_path(MST, layer);
        //IO::write_PLY(argv[4], path);
        //cout << "path done" << endl;
        //pick the closes point to the last point in the previous path as a start point for solving the MST
        paths_list.push_back(path);
    }


    //step 8: link solutions into 1 path
    vector<Eigen::Vector3f > result;
    for (size_t i = 0; i < paths_list.size(); i++)
    {   
        vector<Eigen::Vector3f> path = paths_list[i];
        for (size_t j = 0; j < path.size(); j++)
        {
            result.push_back(path[j]);
        }
    }

    IO::write_PLY(argv[4], result);
    cout << "path done" << endl;


   return EXIT_SUCCESS;
}


