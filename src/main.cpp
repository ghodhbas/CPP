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

bool stitch_segments(vector< vector<Eigen::Vector3f >>& path_segments, SurfaceMesh& surface, Tree& tree, vector<Eigen::Vector3f >& final_path){
    //stitch path segments 
    //float dis = calculate_distance(final_path);

    //select segment closest to  to floor 
    int idx = 0;
    float y_pos = path_segments[0][0].y();
    for (int i = 1; i < path_segments.size(); i++)
    {
        if (path_segments[i][0].y() < y_pos) {
            idx = i;
            y_pos = path_segments[i][0].y();
        }
    }


    //first segment
    for (int i = 0; i < path_segments[idx].size(); i++)
    {
        final_path.push_back(path_segments[idx][i]);
    }
    path_segments.erase(path_segments.begin() + idx);


    //find segment with start point closest to end point of previous segmentee
    while (path_segments.size() > 0) {
        Eigen::Vector3f prev = final_path[final_path.size() - 1];

        float min_distance = 99999999.f;
        idx = 99999;
        for (int i = 0; i < path_segments.size(); i++)
        {
            Eigen::Vector3f start = path_segments[i][0];

            // if no collision
            if (!MyMesh::ray_box_interstction(surface, Eigen::Vector3f(start.x(), start.y(), start.z()), Eigen::Vector3f(prev.x(), prev.y(), prev.z()), tree)) {
                float distance = (prev - start).norm();
                if (distance < min_distance) {
                    idx = i;
                    min_distance = distance;
                }
            }

        }

        if (idx == 99999) {
            cout << "Couldnt find a possible next segment to attatch to " << endl;
            cout << "Abondonning  this trial" << endl;
            return false;
        }

        vector<Eigen::Vector3f > chosen_seg = path_segments[idx];
        for (int i = 0; i < chosen_seg.size(); i++)
        {
            final_path.push_back(chosen_seg[i]);
        }
        path_segments.erase(path_segments.begin() + idx);

    }

    return true;
}

void heapPermutation(vector<int>& a, int size, vector<vector<int>>& all_perms)
{
    // if size becomes 1 then prints the obtained
    // permutation
    if (size == 1) {
        all_perms.push_back(a);
        return;
    }


    heapPermutation(a, size - 1, all_perms);

    for (int i = 0; i < size-1; i++) {
        

        // if size is odd, swap 0th i.e (first) and 
        // (size-1)th i.e (last) element
        if (size % 2 == 0)
            swap(a[i], a[size - 1]);

        // If size is even, swap ith and 
        // (size-1)th i.e (last) element
        else
            swap(a[0], a[size - 1]);
        heapPermutation(a, size - 1, all_perms);
    }
}

bool stitch_segments2(vector< vector<Eigen::Vector3f >>& path_segments, SurfaceMesh& surface, Tree& tree, vector<Eigen::Vector3f >& final_path) {

    //options 1: create midpoint, create graph and make graph

    //option 2: creta graph iwth start and end make distance between start and end 0;


    //option 3: the number of segment is usually low so brute force is fine?
    int nb_nodes = path_segments.size();
    vector<int> node_idx(nb_nodes);
    std::iota(node_idx.begin(), node_idx.end(), 0);

    vector<vector<int>> all_perms;
    heapPermutation(node_idx, nb_nodes,  all_perms);

    vector<float> all_distances;
    float max_distance = 99999999.f;
    bool path_found = false;

    for (int i = 0; i < all_perms.size(); i++)
    {   
        vector<int> perm = all_perms[i];
        vector<Eigen::Vector3f > path;

        //for reach permuation
        bool valid = true;
        for (int j = 0; j < perm.size(); j++)
        {
            //get each segment and check if the next one is attatchable
            vector<Eigen::Vector3f > segment = path_segments[perm[j]];
            
            if (j > 0) {
                // if no collision
                Eigen::Vector3f prev = path[path.size() - 1];
                if (MyMesh::ray_box_interstction(surface, Eigen::Vector3f(segment[0].x(), segment[0].y(), segment[0].z()), Eigen::Vector3f(prev.x(), prev.y(), prev.z()), tree)) {
                    valid = false;
                    break;
                }
            }

            for (int k = 0; k < segment.size(); k++)
            {
                path.push_back(segment[k]);
            }
            
        }

        if (valid) {
            //cout << " VALID PATH FOUND" << endl;
            float dist = calculate_distance(path);
            if (dist < max_distance) {
                path_found = true;
                final_path = path;
            }
        }


    }

    if (path_found)return true;
    return false;
}

template <typename T>
vector<size_t> sort_indexes(const vector<T>& v) {

    // initialize original index locations
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values 
    stable_sort(idx.begin(), idx.end(),
        [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });

    return idx;
}

void method1(Polyhedron& poly, SurfaceMesh& surface, string path_file, float curr_res, int nb_layers, float min_cov, float curr_radius) {
    float near = 1.f ;
    float far = 8.f;
    float Hfov = 120.f;
    float Vfov = 120.f;
    LayeredPP lpp(near, far);

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));


    //Step 2: convert surface mesh into point cloud that has normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);


    //Step 3: voxalize the whole cloud and get cloud - Int he paper this si the volumetric data. / octree
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 0.5f);
    cout << "VOXELGRID SIZE: " << mesh_voxelgrid.getFilteredPointCloud().points.size() << endl;

    //output voxel grid
    pcl::PointCloud<pcl::PointNormal> grid = mesh_voxelgrid.getFilteredPointCloud();
    std::vector< Kernel::Point_3> v_grid_io;
    for (size_t i = 0; i < grid.points.size(); i++)
    {
        v_grid_io.push_back(Kernel::Point_3(grid.points[i].x, grid.points[i].y, grid.points[i].z));
    }
    IO::write_PLY("out/VoxelGrid.ply", v_grid_io);



    //Step 4: generate viewpoints from mesh grid and make them into a filtered point cloud
    typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
    Viewpoints viewpoints = lpp.generate_viewpoints(mesh_voxelgrid, poly);
    pcl::PointCloud<pcl::PointNormal>::Ptr viewpoints_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    //used to check point inside of poly
    //CGAL::Side_of_triangle_mesh<Polyhedron, Kernel> inside(poly);
    for (size_t i = 0; i < viewpoints.size(); i++)
    {
        pcl::PointNormal point = pcl::PointNormal(viewpoints.at(i).first[0], viewpoints.at(i).first[1], viewpoints.at(i).first[2], viewpoints.at(i).second[0], viewpoints.at(i).second[1], viewpoints.at(i).second[2]);
        viewpoints_cloud->push_back(point);
    }
    

    //cout << "Total Number of Viewpoints before filter : " << viewpoints_cloud->points.size() << endl;
    //std::vector< Kernel::Point_3> vpts;
    //for (size_t i = 0; i < viewpoints_cloud->points.size(); i++)
    //{
    //    vpts.push_back(Kernel::Point_3(viewpoints_cloud->points[i].x, viewpoints_cloud->points[i].y, viewpoints_cloud->points[i].z));
    //}
    //
    //string outvpts = "out/viewpoints_before";
    //outvpts += std::string("_").append(std::to_string(curr_res));
    //outvpts += std::string("_").append(std::to_string(curr_radius));
    //IO::write_PLY(std::string(outvpts).append(".ply"), vpts);



    //Step 5: downsample viewepoints    
    float ViewpointvoxelRes = curr_res;
    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> downsampled_viewpoints_grid;
    //allow for normal downsampling with position
    downsampled_viewpoints_grid.setDownsampleAllData(true);
    downsampled_viewpoints_grid.setInputCloud(viewpoints_cloud);
    downsampled_viewpoints_grid.setLeafSize(ViewpointvoxelRes, ViewpointvoxelRes, ViewpointvoxelRes);
    downsampled_viewpoints_grid.initializeVoxelGrid();
    pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_viewpoints = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>(downsampled_viewpoints_grid.getFilteredPointCloud()));

    cout << "Total Number of Viewpoints afterfilter : " << downsampled_viewpoints->points.size() << endl;

    


    //calculate BBox of cloud
    std::pair<pcl::PointXYZ, pcl::PointXYZ>  min_max = MyMesh::get_cloud_BBOX(downsampled_viewpoints);

    //Step 5: split the  viewpoints into layers
    vector< pcl::PointCloud<pcl::PointNormal>::Ptr> layer_viewpoints = lpp.construct_layers(downsampled_viewpoints, nb_layers, min_max);

    //make list of viewpoints per layer

    vector<Viewpoints> viewpoints_list; //*2
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
       


    //step 6: Path generation
    Tree tree(faces(surface).first, faces(surface).second, surface);
    Viewpoints final_viewpoints;

    vector< Viewpoints> all_final_viewpoints;
    vector<Eigen::Vector3f > final_path;

    ExploratoryPlanner::generate_path_layer(layer_viewpoints[0], viewpoints_list[0], grid.makeShared(), near, far, Hfov, Vfov, ViewpointvoxelRes, curr_radius, surface, final_viewpoints, tree, final_path, nullptr);
    all_final_viewpoints.push_back(final_viewpoints);

    //get path for every layer
    Eigen::Vector3f* last_point = &final_path[final_path.size() - 1];
    for (size_t i = 1; i < layer_viewpoints.size(); i++)
    {
        Viewpoints viewpoints;
        if (!ExploratoryPlanner::generate_path_layer(layer_viewpoints[i], viewpoints_list[i], grid.makeShared(), near, far, Hfov, Vfov, ViewpointvoxelRes, curr_radius, surface, viewpoints, tree, final_path, last_point)) {
            return;
        }
        last_point = &final_path[final_path.size() - 1];
        all_final_viewpoints.push_back(viewpoints);
    }



   /****************************************** VALIDATION **************************************************************/
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
            //fc.setInputCloud(cloud);
            fc.setInputCloud(grid.makeShared());
            fc.setVerticalFOV(Vfov);
            fc.setHorizontalFOV(Hfov);
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
                //float angle = std::acos(dot / (dir.norm() * n.norm())) * 180.f / M_PI;

                float angle = std::acos(dot/ std::sqrtf(dir.squaredNorm()* n.squaredNorm())) * 180.f / M_PI;
                //cout << angle << endl;
                if ((180-angle) <= 60.f) {
                    //if (!MyMesh::ray_box_interstction(surface, position, Eigen::Vector3f(p.x, p.y, p.z), tree)) {
                        validation_cloud->points.push_back(p);
                    //}
                       
                }
            }

            //*validation_cloud += *output;
        }
    }
    

    cout << "nb point viewed: " << validation_cloud->points.size() << endl;
    pcl::VoxelGrid<pcl::PointNormal> clean;
    pcl::PointCloud<pcl::PointNormal>::Ptr o(new pcl::PointCloud<pcl::PointNormal>);
    clean.setDownsampleAllData(true);
    clean.setInputCloud(validation_cloud);
    clean.setLeafSize(0.2f, 0.2f, 0.2f);
    clean.filter(*o);
    float coverage = ((float)o->points.size() / grid.points.size()) * 100;
    cout << "Coverage: " << coverage << endl;
    if (coverage < min_cov) return;
    
    float dis = calculate_distance(final_path);
    //calculate path distance
    cout << "TOTAL DISTANCE: " << dis  << endl;
    if (dis > 5100.f) return;

    //output path
    path_file += std::string("_").append(std::to_string(curr_res));
    path_file += std::string("_").append(std::to_string(curr_radius));
    IO::write_PLY(std::string(path_file).append(".ply"), final_path);
    cout << "path done" << endl;

    //calculate path distance
    cout << "TOTAL DISTANCE: " << calculate_distance(final_path) << endl;


    //io observed vertices
    std::vector< Kernel::Point_3> points_test;
    for (size_t i = 0; i < validation_cloud->points.size(); i++)
    {
        points_test.push_back(Kernel::Point_3(validation_cloud->points[i].x, validation_cloud->points[i].y, validation_cloud->points[i].z));
    }
    string out = "out/validation";
    out += std::string("_").append(std::to_string(curr_res));
    out += std::string("_").append(std::to_string(curr_radius));
    IO::write_PLY(std::string(out).append(".ply"), points_test);
}



void method2(Polyhedron& poly, SurfaceMesh& surface, string path_file, float curr_res, int nb_layers, float min_cov, float curr_radius, std::vector<SurfaceMesh*>& segments_vec) {
    float near = 1.0f;
    float far = 8.f;
    float Hfov = 120.f;
    float Vfov = 120.f;
    LayeredPP lpp(near, far);

    Tree tree(faces(surface).first, faces(surface).second, surface);//used for mesh line collision check

    std::vector<SurfaceMesh::Property_map<surface_vertex_descriptor, Vector>> normals_vec;

    vector< vector<Eigen::Vector3f >> path_segments;

    //sort segments
    vector<float> int_order;
    for (int i = 0; i < segments_vec.size(); i++)
    {
        SurfaceMesh* s = segments_vec[i];
        float min = 9999999999.f;
        for (surface_vertex_descriptor vd: s->vertices())
        {
            float y = s->point(vd).y();
            if (y < min)  min = y;
        }

        int_order.push_back(min);
    }

    //std::cout << "index order: ";
    //for (auto i : sort_indexes(int_order)) {
    //    std::cout << " " << i << "= " << int_order[i] << endl;
    //}


    vector< Viewpoints> all_final_viewpoints;
    //construct path for every segment 
    //for (int i = 0; i < segments_vec.size(); i++)
    //iterate through submeshes in vertical order
    for (auto i: sort_indexes(int_order))
    {
        SurfaceMesh seg_surf = *segments_vec[i];
        Polyhedron seg_poly;
        CGAL::copy_face_graph(seg_surf, seg_poly);

        // Step 1 calculate per vertex normals
        std::map<poly_vertex_descriptor, Vector> vnormals;
        CGAL::Polygon_mesh_processing::compute_vertex_normals(seg_poly, boost::make_assoc_property_map(vnormals));

        // Step 2: convert surface mesh into point cloud that has normals
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
        MyMesh::convert_to_pointcloud(seg_poly, cloud, vnormals);


        //Step 3: voxalize the whole cloud and get cloud - Int he paper this si the volumetric data. / octree
        pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 0.5f);
        //cout << "VOXELGRID SIZE: " << mesh_voxelgrid.getFilteredPointCloud().points.size() << endl;

        //output voxel grid
        pcl::PointCloud<pcl::PointNormal> grid = mesh_voxelgrid.getFilteredPointCloud();
        //std::vector< Kernel::Point_3> v_grid_io;
        //for (size_t j = 0; j < grid.points.size(); j++)
        //{
        //    v_grid_io.push_back(Kernel::Point_3(grid.points[j].x, grid.points[j].y, grid.points[j].z));
        //}
        //IO::write_PLY("out/VoxelGrid"+ std::string("_").append(".ply"), v_grid_io);



        //Step 4: generate viewpoints from mesh grid and make them into a filtered point cloud
        typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>  Viewpoints;
        Viewpoints viewpoints_raw = lpp.generate_seg_viewpoints(mesh_voxelgrid, seg_poly, poly);
        pcl::PointCloud<pcl::PointNormal>::Ptr viewpoints_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
        //used to check point inside of poly
        //CGAL::Side_of_triangle_mesh<Polyhedron, Kernel> inside(seg_poly);
        for (size_t j = 0; j < viewpoints_raw.size(); j++)
        {
            pcl::PointNormal point = pcl::PointNormal(viewpoints_raw.at(j).first[0], viewpoints_raw.at(j).first[1], viewpoints_raw.at(j).first[2], viewpoints_raw.at(j).second[0], viewpoints_raw.at(j).second[1], viewpoints_raw.at(j).second[2]);
            viewpoints_cloud->push_back(point);
        }


        //cout << "Total Number of Viewpoints before filter : " << viewpoints_cloud->points.size() << endl;


        //Step 5: downsample viewepoints    
        float ViewpointvoxelRes = curr_res;
        pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> downsampled_viewpoints_grid;
        //allow for normal downsampling with position
        downsampled_viewpoints_grid.setDownsampleAllData(true);
        downsampled_viewpoints_grid.setInputCloud(viewpoints_cloud);
        downsampled_viewpoints_grid.setLeafSize(ViewpointvoxelRes, ViewpointvoxelRes, ViewpointvoxelRes);
        downsampled_viewpoints_grid.initializeVoxelGrid();
        pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_viewpoints = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>(downsampled_viewpoints_grid.getFilteredPointCloud()));

        //cout << "Total Number of Viewpoints afterfilter : " << downsampled_viewpoints->points.size() << endl;
        

        //setup viewpoint format
        Viewpoints viewpoints;
        for (size_t j = 0; j < downsampled_viewpoints->points.size(); j++)
        {
            pcl::PointNormal point = downsampled_viewpoints->at(j);
            //std::pair<Eigen::Vector3f, Eigen::Vector3f> tmp = std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z));
            viewpoints.push_back(std::pair< Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f(point.x, point.y, point.z), Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z)));
        }

        // step 6: Path generation


        Viewpoints final_viewpoints;

        vector<Eigen::Vector3f > seg_path;

        ExploratoryPlanner::generate_path_layer(downsampled_viewpoints, viewpoints, grid.makeShared(), near, far, Hfov, Vfov, ViewpointvoxelRes, curr_radius, surface, final_viewpoints, tree, seg_path, nullptr);
        all_final_viewpoints.push_back(final_viewpoints);

        //std::string p_f = path_file;
        //p_f += std::string("_").append(std::to_string(i+1));
        //IO::write_PLY(std::string(p_f).append(".ply"), seg_path);
        //cout << "path  segment done" << endl;

        path_segments.push_back(seg_path);
    }


    vector<Eigen::Vector3f > final_path;
    if (!stitch_segments2(path_segments, surface, tree, final_path)) {
        cout << "Path not found" << endl;
        return;
   }
    


    /****************************************** VALIDATION **************************************************************/

    // Step 1 calculate per vertex normals
    std::map<poly_vertex_descriptor, Vector> vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly, boost::make_assoc_property_map(vnormals));
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>);
    MyMesh::convert_to_pointcloud(poly, cloud, vnormals);

    pcl::VoxelGridOcclusionEstimation<pcl::PointNormal> mesh_voxelgrid = lpp.voxelize(cloud, 0.5f);
    cout << "VOXELGRID SIZE: " << mesh_voxelgrid.getFilteredPointCloud().points.size() << endl;

    //output voxel grid
    pcl::PointCloud<pcl::PointNormal> grid = mesh_voxelgrid.getFilteredPointCloud();
    std::vector< Kernel::Point_3> v_grid_io;
    for (size_t i = 0; i < grid.points.size(); i++)
    {
        v_grid_io.push_back(Kernel::Point_3(grid.points[i].x, grid.points[i].y, grid.points[i].z));
    }
    IO::write_PLY("out/VoxelGrid.ply", v_grid_io);

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
            //fc.setInputCloud(cloud);
            fc.setInputCloud(grid.makeShared());
            fc.setVerticalFOV(Vfov);
            fc.setHorizontalFOV(Hfov);
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
                //float angle = std::acos(dot / (dir.norm() * n.norm())) * 180.f / M_PI;

                float angle = std::acos(dot / std::sqrtf(dir.squaredNorm() * n.squaredNorm())) * 180.f / M_PI;
                //cout << angle << endl;
                if ((180 - angle) <= 60.f) {
                    //if (!MyMesh::ray_box_interstction(surface, position, Eigen::Vector3f(p.x, p.y, p.z), tree)) {
                    validation_cloud->points.push_back(p);
                    //}

                }
            }

            //*validation_cloud += *output;
        }
    }


    cout << "nb point viewed: " << validation_cloud->points.size() << endl;
    pcl::VoxelGrid<pcl::PointNormal> clean;
    pcl::PointCloud<pcl::PointNormal>::Ptr o(new pcl::PointCloud<pcl::PointNormal>);
    clean.setDownsampleAllData(true);
    clean.setInputCloud(validation_cloud);
    clean.setLeafSize(0.2f, 0.2f, 0.2f);
    clean.filter(*o);
    float coverage = ((float)o->points.size() / grid.points.size()) * 100;
    cout << "Coverage: " << coverage << endl;
    if (coverage < min_cov) return;

    float dis = calculate_distance(final_path);
    //calculate path distance
    cout << "TOTAL DISTANCE: " << dis << endl;
    if (dis > 5700.f) return;

    //output path
    path_file += std::string("_").append(std::to_string(curr_res));
    path_file += std::string("_").append(std::to_string(curr_radius));
    IO::write_PLY(std::string(path_file).append(".ply"), final_path);
    cout << "path done" << endl;

    //calculate path distance
    cout << "TOTAL DISTANCE: " << calculate_distance(final_path) << endl;


    //io observed vertices
    std::vector< Kernel::Point_3> points_test;
    for (size_t i = 0; i < validation_cloud->points.size(); i++)
    {
        points_test.push_back(Kernel::Point_3(validation_cloud->points[i].x, validation_cloud->points[i].y, validation_cloud->points[i].z));
    }
    string out = "out/validation";
    out += std::string("_").append(std::to_string(curr_res));
    out += std::string("_").append(std::to_string(curr_radius));
    IO::write_PLY(std::string(out).append(".ply"), points_test);


    return;
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


    string path_file = argv[3];
    float min_res = strtof(argv[4], NULL);
    float max_res = strtof(argv[5], NULL);
    float incr_res = strtof(argv[6], NULL);
    int nb_layers = strtof(argv[7], NULL);
    float min_radius = strtof(argv[8], NULL);
    float max_radius = strtof(argv[9], NULL);
    float incr_radius = strtof(argv[10], NULL);
    float min_cov = 97.f;
    
    if (method._Equal("1")) {
        float curr_res = min_res; 
        float curr_radius = min_radius;
        while(curr_res<=max_res){

            std::cout << "-------------------------- ResolLution: " << curr_res << " -----------------------" << std::endl;
            while (curr_radius <= max_radius) {
                auto start = std::chrono::high_resolution_clock::now();
                std::cout << "             ----------------- Radius:  " << curr_radius << " ---------            " << std::endl;
                method1(poly, surface, path_file, curr_res, nb_layers, min_cov, curr_radius);
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                std::cout << "DURATION: " << duration.count() << endl;
                curr_radius += incr_radius;
            }
            
            curr_res += incr_res;
            curr_radius = min_radius;
        }

    }


    if (method._Equal("2")) {
        float curr_res = min_res;
        float curr_radius = min_radius;
        std::vector<SurfaceMesh*> segments_vec;
        //segment surface into segemnts
        auto start = std::chrono::high_resolution_clock::now();
        MyMesh::segment_mesh(surface, segments_vec);
        //MyMesh::skeleton_segment_mesh(poly, segments_vec);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "DURATION: " << duration.count() << endl;
        

        while (curr_res <= max_res) {

            std::cout << "-------------------------- ResolLution: " << curr_res << " -----------------------" << std::endl;
            while (curr_radius <= max_radius) {
                auto start = std::chrono::high_resolution_clock::now();
                std::cout << "             ----------------- Radius:  " << curr_radius << " ---------            " << std::endl;
                method2(poly, surface, path_file, curr_res, nb_layers, min_cov, curr_radius, segments_vec);
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                std::cout << "DURATION: " << duration.count() << endl;
                curr_radius += incr_radius;
            }

            curr_res += incr_res;
            curr_radius = min_radius;
        }

    }
   


   return EXIT_SUCCESS;
}
