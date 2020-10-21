#pragma once

#ifndef MYMESH
#define MYMESH

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "CGAL_includes.h"
#include <pcl/filters/voxel_grid.h>
#include "IO.h"

namespace MyMesh {

   void print_mesh_info(Polyhedron& m);
  
   void load_verticies_and_color(SurfaceMesh surface, std::vector<Point>& verts, std::vector<CGAL::Color>& cols);
  
   void color_surface(SurfaceMesh& surface, int r, int g, int b, int a);

   void color_visible_surface(pcl::PointCloud<pcl::PointXYZ> visible_s, SurfaceMesh& surface);
  
   void segment_mesh(SurfaceMesh surface);
  
   void convert_to_pointcloud(SurfaceMesh& surface, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  
   //constructs point clound and returns the max and min values for all points (can be used for bounding box)
   std::pair<pcl::PointXYZ, pcl::PointXYZ>  convert_to_pointcloud(Polyhedron& poly, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, std::map<poly_vertex_descriptor, Vector>& vnormals);

   double max_coordinate(const Polyhedron& poly);

   bool point_inside_mesh(CGAL::Side_of_triangle_mesh<Polyhedron, Kernel>& inside, Eigen::Vector4f point_eigen);

   void remove_points_inside_mesh(Polyhedron poly, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& viewpoints, std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>>>& final_viewpoints, std::vector< Kernel::Point_3>& viewpoints_cloud);
    

}

#endif

