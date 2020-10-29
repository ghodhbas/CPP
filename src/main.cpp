#include <pcl/point_types.h>
#include "Occlusion_Culling.h"
#include <fstream>
#include <iostream>
#include <cmath> 
#include <ctime>
#include "PathPlanner.h"
#include "IO.h"
#include "LayeredPP.h"
#include <pcl/filters/frustum_culling.h>

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







void method2(Polyhedron& poly, SurfaceMesh& surface, char* argv[]) {
    LayeredPP lpp(0.1f, 5.5f);

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));


    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);


    //Step 3: voxalize the whole cloud and get cloud - Int he paper this si the volumetric data.
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 2.f);


    //Step 4: generate viewpoints from mesh grid and make them into a filtered point cloud
    typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
    Viewpoints viewpoints = lpp.generate_viewpoints(mesh_voxelgrid);
    pcl::PointCloud<pcl::PointNormal>::Ptr viewpoints_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    for (size_t i = 0; i < viewpoints.size(); i++)
    {

        pcl::PointNormal point = pcl::PointNormal(viewpoints.at(i).first[0], viewpoints.at(i).first[1], viewpoints.at(i).first[2], viewpoints.at(i).second[0], viewpoints.at(i).second[1], viewpoints.at(i).second[2]);
        viewpoints_cloud->push_back(point);
    }
    //calculate BBox of cloud
    std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max = MyMesh::get_cloud_BBOX(viewpoints_cloud);



    //Step 5: split the  viewpoints into layers
    int nb_layers = 4;
    vector< pcl::PointCloud<pcl::PointNormal>::Ptr> layer_viewpoints = lpp.construct_layers(viewpoints_cloud, nb_layers, min_max);


    //Step 6: Voxelize every layer of viewpoints and make input for TSP (reduce number of viewpoints)
    vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> viewpoints_voxel_layers = lpp.voxelize_layers(layer_viewpoints, 1.f);
    vector<Viewpoints> downsampled_viewpoints_perlayer;
    for (int i = 0; i < viewpoints_voxel_layers.size(); i++) {
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered_layer_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>(viewpoints_voxel_layers[i].getFilteredPointCloud()));
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
        std::map<int, std::map<int, float>> distance_map = lpp.calculate_distances(layer, pair_vec, surface);
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
    vector<Eigen::Vector3f > final_path;

    Eigen::Vector3f last;
    int last_idx;
    int x = 0;
    for (size_t i = 0; i < paths_list.size(); i++)
    {
        vector<Eigen::Vector3f> path = paths_list[i];
        int idx = -1;
        float d = 9999999.f;
        for (size_t j = 0; j < path.size(); j++)
        {
            final_path.push_back(path[j]);

            ////calculate distance last point to other point the next layer
            //if (i > 0) {
            //    if (d > (path[j] - last).norm()) {
            //        idx = x;
            //        d = (path[j] - last).norm();
            //    }
            //}

            x++;
        }

        //
        //if (idx > -1) {
        //    Eigen::Vector3f p = final_path[idx];
        //    final_path.erase(final_path.begin()+idx);
        //    final_path.emplace(final_path.begin() + last_idx+1, p);
        //}
        //
        //last = final_path[x-1];
        //last_idx = x - 1;


    }



    IO::write_PLY(argv[4], final_path);
    cout << "path done" << endl;


    //validationcloud output
    pcl::PointCloud<pcl::PointNormal>::Ptr validation_cloud(new pcl::PointCloud<pcl::PointNormal>);

    //test for completeneness
    pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointNormal>);   // using volumetric data instead of actual mesh  ---- can switch and test on actual mesh
    *voxel_cloud = mesh_voxelgrid.getFilteredPointCloud();
    //follow each node
    for (size_t i = 0; i < downsampled_viewpoints_perlayer.size(); i++)
    {
        Viewpoints layer = downsampled_viewpoints_perlayer[i];

 
        // retrieve the viewpoint and the direction
        for (size_t j = 0; j < layer.size(); j++)
        {

            std::pair<Eigen::Vector3f, Eigen::Vector3f> viewpoint = layer[j];
            Eigen::Vector3f position = viewpoint.first;
            Eigen::Vector3f dir = viewpoint.second;
            dir.normalize();
            pcl::FrustumCulling<pcl::PointNormal> fc;
            //set either the input cloud or voxelized cloud
            fc.setInputCloud(cloud);
            //fc.setInputCloud(voxel_cloud);
            fc.setVerticalFOV(60);
            fc.setHorizontalFOV(90);
            fc.setNearPlaneDistance(lpp.get_near_plane_d());
            fc.setFarPlaneDistance(lpp.get_far_plane_d());

            Eigen::Matrix4f pose;
             // /* Find cospi and sinpi */
             float c1 = sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
             float s1 = dir[2];
             ///* Find cosO and sinO; if gimbal lock, choose (1,0) arbitrarily */
             float c2 = c1 ? dir[0] / c1 : 1.0;
             float s2 = c1 ? dir[1] / c1 : 0.0;
             //
             //return mat3(v.x, -s2, -s1 * c2,
             //    v.y, c2, -s1 * s2,
             //    v.z, 0, c1);
             pose << dir[0], -s2, -s1 * c2, position[0],
                     dir[1], c2, -s1 * s2, position[1],
                     dir[2], 0, c1, position[2],
                     0.f, 0.f, 0.f, 1.f; 



            fc.setCameraPose(pose);
            
            pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
            fc.filter(*output);
            
            //go through the points and verify angle threshhold before adding
            for (size_t k= 0; k < output->points.size(); k++)
            {   
                pcl::PointNormal p = output->points[k];
                Eigen::Vector3f n(p.normal_x, p.normal_y, p.normal_z);
                n.normalize();
                float dot = dir.dot(n);
                float angle = std::acos(dot / (dir.norm() * n.norm()));
                if (angle < 45.f) {
                    validation_cloud->points.push_back(p);
                }
            }

            //*validation_cloud += *output;
        }


    }

    //IO voxelcloud
    std::vector< Kernel::Point_3> points_voxel;
    for (size_t i = 0; i < voxel_cloud->points.size(); i++)
    {
        points_voxel.push_back(Kernel::Point_3(voxel_cloud->points[i].x, voxel_cloud->points[i].y, voxel_cloud->points[i].z));
    }
    IO::write_PLY("out/source.ply", points_voxel);



    //io observed vertices
    std::vector< Kernel::Point_3> points_test;
    for (size_t i = 0; i < validation_cloud->points.size(); i++)
    {
        points_test.push_back(Kernel::Point_3(validation_cloud->points[i].x, validation_cloud->points[i].y, validation_cloud->points[i].z));
    }
    IO::write_PLY("out/validation.ply", points_test);
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
    method2(poly, surface, argv);


   return EXIT_SUCCESS;
}


