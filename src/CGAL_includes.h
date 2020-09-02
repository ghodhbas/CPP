#pragma once

//Setting
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

//GEOMETRY
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Cartesian.h>

//GEometry propertiy
#include <CGAL/IO/Color.h>
#include <CGAL/property_map.h>

//IO
//#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/IO/PLY.h>

//used to get mesh datat
#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>

//treat floating points exactly
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::array<unsigned char, 4> MyColor;

// Point with color 
typedef std::tuple<Point, MyColor> PCI;
typedef CGAL::Nth_of_tuple_property_map<0, PCI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PCI> Color_map;