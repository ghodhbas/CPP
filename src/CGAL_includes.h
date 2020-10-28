#pragma once
#include <CGAL/config.h>

//Setting
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

//GEOMETRY
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Cartesian.h>

//GEometry propertiy
#include <CGAL/IO/Color.h>
#include <CGAL/property_map.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

//IO
//#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/IO/PLY.h>

//used to get mesh datat
#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>
#include <CGAL/mesh_segmentation.h>

//boost
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/boost/graph/copy_face_graph.h>


//ray box inter
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

//treat floating points exactly
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Ray_3 Ray;

typedef std::array<unsigned char, 4> MyColor;

// Point with color 
typedef std::tuple<Point, MyColor> PCI;
typedef CGAL::Nth_of_tuple_property_map<0, PCI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PCI> Color_map;

//descriptors

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor surface_vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor surface_face_descriptor;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor poly_vertex_descriptor;
typedef boost::graph_traits<Polyhedron>::face_descriptor   poly_face_descriptor;
typedef Polyhedron::Vertex_iterator        poly_vertex_iterator;

//ray box
typedef CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;
