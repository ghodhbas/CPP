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
#include <chrono> 
#include "ExploratoryPlanner.h"
using namespace std;

float calculate_distance(vector<Eigen::Vector3f > path) {

    float total = 0.f;
    Eigen::Vector3f last = path[0];
    for (size_t i = 1; i < path.size(); i++)
    {
        total += (path[i] - last).norm();
        last = path[i];
    }
    return total;
}


/* METHOD 1: Set cover + TSP  */
void method_1(Polyhedron& poly, SurfaceMesh& surface, char* argv[]) {
    //make point cloud from vertices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    MyMesh::convert_to_pointcloud(surface, cloud);
    cout << "Number of points in the Point cloud: " << cloud->points.size() << endl << endl;


    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));


    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_n = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud_with_n, vnormals);




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



    //saves the indices withion the graph where the viewpoints are located
    cout << "Graph Construction from Voxel Grid..." << endl;
    std::vector<int> viewpoint_graph_idx;
    pp.construct_graph(poly, Solution, viewpoint_graph_idx);
    cout << "Graph Construction Complete" << endl<< endl;
    //construct all the pairs from the viewpoints and calculate the distances between them -- edge list
    cout << "Calculating shortest distances between pairs of viewpoints..." << endl;
    vector<std::pair<int, int>> pair_vec;
    std::map<int, std::map<int, int>> distance_map  =  pp.calculate_distances(viewpoint_graph_idx, pair_vec, surface);
    cout << "Distances calculation Complete" << endl << endl;

    cout << "Constructing Minimun Spanning tree between solution viewpoints..." << endl;
    Edge_Graph MST = pp.construct_MST(pair_vec, distance_map);
    cout << "MST CONSTRUCTED" << endl<<endl;

    vector<Eigen::Vector3f > path = pp.generate_path(MST);
    IO::write_PLY(argv[3], path);

    //calculate path distance
    cout << "TOTAL DISTANCE: " << calculate_distance(path);






    //validationcloud output
    pcl::PointCloud<pcl::PointNormal>::Ptr validation_cloud(new pcl::PointCloud<pcl::PointNormal>);

    //test for completeneness
    //follow each node
    pcl::FrustumCulling<pcl::PointNormal> fc;
    //set either the input cloud or voxelized cloud
    fc.setInputCloud(cloud_with_n);
    //fc.setInputCloud(voxel_cloud);
    fc.setVerticalFOV(90);
    fc.setHorizontalFOV(100);
    fc.setNearPlaneDistance(0.5f);
    fc.setFarPlaneDistance(3.f);
    for (size_t i = 0; i < Solution.size(); i++)
    {
        Eigen::Matrix4f pose = Solution[i].first;
        fc.setCameraPose(pose);
        pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
        fc.filter(*output);

        //go through the points and verify angle threshhold before adding
        for (size_t k = 0; k < output->points.size(); k++)
        {
            pcl::PointNormal p = output->points[k];
            Eigen::Vector3f n(p.normal_x, p.normal_y, p.normal_z);
            n.normalize();
            Eigen::Vector3f dir = pose.block(0, 0, 3, 1);
            float dot = dir.dot(n);
            float angle = std::acos(dot / (dir.norm() * n.norm()));
            if (angle < 70.f) {
                validation_cloud->points.push_back(p);
            }
        }
        
    }


    //io observed vertices
    std::vector< Kernel::Point_3> points_test;
    for (size_t i = 0; i < validation_cloud->points.size(); i++)
    {
        points_test.push_back(Kernel::Point_3(validation_cloud->points[i].x, validation_cloud->points[i].y, validation_cloud->points[i].z));
    }
    IO::write_PLY("out/validation.ply", points_test);


}




void method2(Polyhedron& poly, SurfaceMesh& surface, char* argv[]) {
    LayeredPP lpp(0.5f, 5.f);

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));


    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);


    //Step 3: voxalize the whole cloud and get cloud - Int he paper this si the volumetric data.
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 3.f);
    cout << "VOXELGRID SIZE: " << mesh_voxelgrid.getFilteredPointCloud().points.size() << endl;


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
    int nb_layers = 5;
    vector< pcl::PointCloud<pcl::PointNormal>::Ptr> layer_viewpoints = lpp.construct_layers(viewpoints_cloud, nb_layers, min_max);


    //Step 6: Voxelize every layer of viewpoints and make input for TSP (reduce number of viewpoints)
    vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> viewpoints_voxel_layers = lpp.voxelize_layers(layer_viewpoints, 7.f);
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
    Eigen::Vector3f last_node(0.f,0.f,0.f);
    //yy increment in layer
    min_max = MyMesh::get_cloud_BBOX(cloud);
    float y_incr = (min_max.second.y- min_max.first.y)/(float)nb_layers;

    Tree tree(faces(surface).first, faces(surface).second, surface);

    for (size_t i = 0; i < downsampled_viewpoints_perlayer.size(); i++)
    {   
        //make list of voxels from mesh in the layer
        //get upper and lower
        float lower = min_max.first.y + y_incr * i ;
        float upper = min_max.first.y + y_incr * (i+1);
        pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            pcl::PointNormal point = cloud->at(i);
            if (point.y >= lower && point.y <= upper) layer_cloud->points.push_back(point);
        }
        std::pair<pcl::PointXYZ, pcl::PointXYZ> layer_bbox = MyMesh::get_cloud_BBOX(layer_cloud);

        //construct list of viewpoints  for that layer
        Viewpoints layer = downsampled_viewpoints_perlayer[i];
        vector<std::pair<int, int>> pair_vec;
        std::map<int, std::map<int, float>> distance_map = lpp.calculate_distances(layer, pair_vec, surface, layer_bbox, tree);
        //cout << "Constructing Minimun Spanning tree between solution viewpoints..." << endl;

        Edge_Graph MST = lpp.construct_MST(pair_vec, distance_map);
        //cout << "MST CONSTRUCTED" << endl << endl;
        //select the node that is the closest to the end of the previous path
        vector<Eigen::Vector3f > path = lpp.generate_path(MST, layer, last_node);
        //IO::write_PLY(argv[3], path);
        //cout << "path done" << endl;
        //pick the closes point to the last point in the previous path as a start point for solving the MST
        paths_list.push_back(path);
        last_node = path[path.size() - 1];

    }


    //step 8: link solutions into 1 path
    vector<Eigen::Vector3f > final_path;

    Eigen::Vector3f last;
    int last_idx;
    int x = 0;
    for (size_t i = 0; i < paths_list.size(); i++)
    {
        vector<Eigen::Vector3f> path = paths_list[i];
        //IO::write_PLY(std::string("out/path_").append(std::to_string(i)+".ply"), path);
        int idx = -1;
        //float d = 9999999.f;
        for (size_t j = 0; j < path.size(); j++)
        {
            final_path.push_back(path[j]);
            x++;
        }
    }



    IO::write_PLY(argv[3], final_path);
    cout << "path done" << endl;


    //calculate path distance
    cout << "TOTAL DISTANCE: " << calculate_distance(final_path)<<endl;


        //validationcloud output
        pcl::PointCloud<pcl::PointNormal>::Ptr validation_cloud(new pcl::PointCloud<pcl::PointNormal>);
    
        
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
                fc.setVerticalFOV(90);
                fc.setHorizontalFOV(110);
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
                    if (angle < 70.f) {
                        validation_cloud->points.push_back(p);
                    }
                }
    
                //*validation_cloud += *output;
            }
    
   
         }
   
    //IO voxelcloud
    //test for completeneness
    //pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointNormal>);   // using volumetric data instead of actual mesh  ---- can switch and test on actual mesh
    //*voxel_cloud = mesh_voxelgrid.getFilteredPointCloud();
    //std::vector< Kernel::Point_3> points_voxel;
    //for (size_t i = 0; i < voxel_cloud->points.size(); i++)
    //{
    //    points_voxel.push_back(Kernel::Point_3(voxel_cloud->points[i].x, voxel_cloud->points[i].y, voxel_cloud->points[i].z));
    //}
    //IO::write_PLY("out/source.ply", points_voxel);
   
   
   
    //io observed vertices
    std::vector< Kernel::Point_3> points_test;
    for (size_t i = 0; i < validation_cloud->points.size(); i++)
    {
        points_test.push_back(Kernel::Point_3(validation_cloud->points[i].x, validation_cloud->points[i].y, validation_cloud->points[i].z));
    }
    IO::write_PLY("out/validation.ply", points_test);
}




void method3(Polyhedron& poly, SurfaceMesh& surface, char* argv[]) {
    LayeredPP lpp(0.5f, 4.5f);

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));


    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);


    //Step 3: voxalize the whole cloud and get cloud - Int he paper this si the volumetric data. / octree
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 3.f);
    cout << "VOXELGRID SIZE: " << mesh_voxelgrid.getFilteredPointCloud().points.size() << endl;


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



    //Step 5: downsample viewepoints    
    float ViewpointvoxelRes = 5.f;
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> downsampled_viewpoints_grid;
    //allow for normal downsampling with position
    downsampled_viewpoints_grid.setDownsampleAllData(true);
    downsampled_viewpoints_grid.setInputCloud(viewpoints_cloud);
    downsampled_viewpoints_grid.setLeafSize(ViewpointvoxelRes, ViewpointvoxelRes, ViewpointvoxelRes);
    downsampled_viewpoints_grid.initializeVoxelGrid();
    pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_viewpoints = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>(downsampled_viewpoints_grid.getFilteredPointCloud()));


    //populate list of viewpoints:  location - direction pair.
    Viewpoints viewpoints_list;
    for (size_t j = 0; j < downsampled_viewpoints->points.size(); j++)
    {
        pcl::PointNormal point = downsampled_viewpoints->at(j);
        //std::pair<Eigen::Vector3f, Eigen::Vector3f> tmp = std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z));
        viewpoints_list.push_back(std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z)));
    }
  
    
    //sort viewpoints by height
    std::sort(downsampled_viewpoints->points.begin(), downsampled_viewpoints->points.end(), Viewpoint_Comparator());
    std::sort(viewpoints_list.begin(), viewpoints_list.end(), Viewpoint_Comparator());
    
    
    Viewpoints final_viewpoints;
    vector<Eigen::Vector3f > final_path = ExploratoryPlanner::generate_path(downsampled_viewpoints, viewpoints_list, surface , final_viewpoints);




    IO::write_PLY(argv[3], final_path);
    cout << "path done" << endl;
    
    //calculate path distance
    cout << "TOTAL DISTANCE: " << calculate_distance(final_path) << endl;
    
    pcl::PointCloud<pcl::PointNormal>::Ptr validation_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    // retrieve the viewpoint and the direction
    for (size_t j = 0; j < final_viewpoints.size(); j++)
    {

        std::pair<Eigen::Vector3f, Eigen::Vector3f> viewpoint = final_viewpoints[j];
        Eigen::Vector3f position = viewpoint.first;
        Eigen::Vector3f dir = viewpoint.second;
        dir.normalize();
        pcl::FrustumCulling<pcl::PointNormal> fc;
        //set either the input cloud or voxelized cloud
        fc.setInputCloud(cloud);
        //fc.setInputCloud(voxel_cloud);
        fc.setVerticalFOV(90);
        fc.setHorizontalFOV(110);
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
        for (size_t k = 0; k < output->points.size(); k++)
        {
            pcl::PointNormal p = output->points[k];
            Eigen::Vector3f n(p.normal_x, p.normal_y, p.normal_z);
            n.normalize();
            float dot = dir.dot(n);
            float angle = std::acos(dot / (dir.norm() * n.norm()));
            if (angle < 70.f) {
                validation_cloud->points.push_back(p);
            }
         }

         //*validation_cloud += *output;
     }


    //io observed vertices
    std::vector< Kernel::Point_3> points_test;
    for (size_t i = 0; i < validation_cloud->points.size(); i++)
    {
        points_test.push_back(Kernel::Point_3(validation_cloud->points[i].x, validation_cloud->points[i].y, validation_cloud->points[i].z));
    }
    IO::write_PLY("out/validation.ply", points_test);
}




void method4(Polyhedron& poly, SurfaceMesh& surface, char* argv[]) {
    LayeredPP lpp(0.5f, 4.f);
    int nb_layers = 6;

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));


    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);


    //Step 3: voxalize the whole cloud and get cloud - Int he paper this si the volumetric data. / octree
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 3.f);
    cout << "VOXELGRID SIZE: " << mesh_voxelgrid.getFilteredPointCloud().points.size() << endl;



    //Step 4: generate viewpoints from mesh grid and make them into a filtered point cloud
    typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
    Viewpoints viewpoints = lpp.generate_viewpoints(mesh_voxelgrid);
    pcl::PointCloud<pcl::PointNormal>::Ptr viewpoints_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    for (size_t i = 0; i < viewpoints.size(); i++)
    {

        pcl::PointNormal point = pcl::PointNormal(viewpoints.at(i).first[0], viewpoints.at(i).first[1], viewpoints.at(i).first[2], viewpoints.at(i).second[0], viewpoints.at(i).second[1], viewpoints.at(i).second[2]);
        viewpoints_cloud->push_back(point);
    }
    

    //Step 5: downsample viewepoints    
    float ViewpointvoxelRes = 4.5f;
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> downsampled_viewpoints_grid;
    //allow for normal downsampling with position
    downsampled_viewpoints_grid.setDownsampleAllData(true);
    downsampled_viewpoints_grid.setInputCloud(viewpoints_cloud);
    downsampled_viewpoints_grid.setLeafSize(ViewpointvoxelRes, ViewpointvoxelRes, ViewpointvoxelRes);
    downsampled_viewpoints_grid.initializeVoxelGrid();
    pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_viewpoints = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>(downsampled_viewpoints_grid.getFilteredPointCloud()));

    //calculate BBox of cloud
    std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max = MyMesh::get_cloud_BBOX(downsampled_viewpoints);

    //Step 5: split the  viewpoints into layers
    //vector< pcl::PointCloud<pcl::PointNormal>::Ptr> layer_viewpoints = lpp.construct_layers(viewpoints_cloud, nb_layers, min_max);
    vector< pcl::PointCloud<pcl::PointNormal>::Ptr> layer_viewpoints = lpp.construct_layers(downsampled_viewpoints, nb_layers, min_max);

    ////Step 6: downsample everylayer 
    //vector<pcl::VoxelGridOcclusionEstimation<pcl::PointNormal>> viewpoints_voxel_layers = lpp.voxelize_layers(layer_viewpoints, 30.f); //*1

    //make list of viewpoints per layer
    //vector<pcl::PointCloud<pcl::PointNormal>::Ptr> downsampled_viewpoints;

    vector<Viewpoints> viewpoints_list; //*2
    //for (int i = 0; i < viewpoints_voxel_layers.size(); i++) {
    //    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_layer_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>(viewpoints_voxel_layers[i].getFilteredPointCloud()));
    //    downsampled_viewpoints.push_back(filtered_layer_cloud);
        //pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> voxel_layer = viewpoints_voxel_layers[i];
    for (int i = 0; i < layer_viewpoints.size(); i++) {
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered_layer_cloud = layer_viewpoints[i];
        Viewpoints layer;
        for (size_t j = 0; j < filtered_layer_cloud->points.size(); j++)
        {
            pcl::PointNormal point = filtered_layer_cloud->at(j);
            //std::pair<Eigen::Vector3f, Eigen::Vector3f> tmp = std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z));
            layer.push_back(std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z)));
        }
        //layer_viewpoints.push_back(layer);
        viewpoints_list.push_back(layer);
    }



    //sort viewpoints by height for first layer
    std::sort(layer_viewpoints[0]->points.begin(), layer_viewpoints[0]->points.end(), Viewpoint_Comparator()); //1
    std::sort(viewpoints_list[0].begin(), viewpoints_list[0].end(), Viewpoint_Comparator());//2\
    
    Tree tree(faces(surface).first, faces(surface).second, surface);
    Viewpoints final_viewpoints;

    vector< Viewpoints> all_final_viewpoints;
    vector<Eigen::Vector3f > final_path;
    ExploratoryPlanner::generate_path_layer(layer_viewpoints[0], viewpoints_list[0], surface,  final_viewpoints, tree, final_path);
    all_final_viewpoints.push_back(final_viewpoints);



    //get path for every layer

    Eigen::Vector3f* last_point = &final_path[final_path.size() - 1];
    for (size_t i = 1; i < layer_viewpoints.size(); i++)
    {
        Viewpoints viewpoints;
        ExploratoryPlanner::generate_path_layer(layer_viewpoints[i], viewpoints_list[i], surface, viewpoints, tree,  final_path, last_point );
        last_point = &final_path[final_path.size() - 1];
        all_final_viewpoints.push_back(viewpoints);
    }




    IO::write_PLY(argv[3], final_path);
    cout << "path done" << endl;

    //calculate path distance
    cout << "TOTAL DISTANCE: " << calculate_distance(final_path) << endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr validation_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    // retrieve the viewpoint and the direction
    for (size_t i = 0; i < all_final_viewpoints.size(); i++)
    {
        Viewpoints final_viewpoints = all_final_viewpoints[i];
        for (size_t j = 0; j < final_viewpoints.size(); j++)
        {

            std::pair<Eigen::Vector3f, Eigen::Vector3f> viewpoint = final_viewpoints[j];
            Eigen::Vector3f position = viewpoint.first;
            Eigen::Vector3f dir = viewpoint.second;
            dir.normalize();
            pcl::FrustumCulling<pcl::PointNormal> fc;
            //set either the input cloud or voxelized cloud
            fc.setInputCloud(cloud);
            //fc.setInputCloud(voxel_cloud);
            fc.setVerticalFOV(90);
            fc.setHorizontalFOV(110);
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
            for (size_t k = 0; k < output->points.size(); k++)
            {
                pcl::PointNormal p = output->points[k];
                Eigen::Vector3f n(p.normal_x, p.normal_y, p.normal_z);
                n.normalize();
                float dot = dir.dot(n);
                float angle = std::acos(dot / (dir.norm() * n.norm()));
                if (angle < 70.f) {
                    validation_cloud->points.push_back(p);
                }
            }

            //*validation_cloud += *output;
        }
    }
    


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
    cout << " argv[2]: " << argv[2] << endl;
    string method = argv[2];
    /*--------------------------------  METHOD 1: Set cover + TSP --------           START FINDING PATH ALGORITHM    ---------------------          */
    if (method._Equal("0")) {

        auto start = std::chrono::high_resolution_clock::now();
        method_1(poly, surface, argv);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        cout << "DURATION: " << duration.count() << endl;
    }
    /*--------------------------------  METHOD 2: layer +normal construction & TSP --------           START FINDING PATH ALGORITHM    ---------------------          */
  
    if (method._Equal("1")){

        auto start = std::chrono::high_resolution_clock::now();
        method2(poly, surface, argv);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "DURATION: " << duration.count() << endl;

    }

    /*--------------------------------  METHOD 3: Exploratory with huristic--------           START FINDING PATH ALGORITHM    ---------------------          */
    if (method._Equal("2")) {
        auto start = std::chrono::high_resolution_clock::now();
        method3(poly, surface, argv);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "DURATION: " << duration.count() << endl;

    }


    if (method._Equal("3")) {
        auto start = std::chrono::high_resolution_clock::now();
        method4(poly, surface, argv);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "DURATION: " << duration.count() << endl;

    }
   


   return EXIT_SUCCESS;
}


