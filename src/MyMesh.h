#pragma once
#include <iostream>
#include <string>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

using namespace std;

//treat floating points exactly
namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Point_3 Point;

void print_mesh_info(Polyhedron& m) {
    //determine mesh type
    string meshType;
    if (m.is_pure_triangle()) {
        meshType = string("Triangle");
    }
    else if (m.is_pure_quad()) {
        meshType = string("Quad");
    }
    else {
        meshType = string("General");
    }


    //loop over verticies
    double valenceSum = 0;
    int minValence = -1;
    int maxValence = -1;
    //for each vertex
    for (Polyhedron::Vertex_const_iterator vertexIter = m.vertices_begin(); vertexIter != m.vertices_end(); ++vertexIter) {

        //get valence
        int valence = vertexIter->degree();

        //update min and max
        if (minValence < 0 || valence < minValence) minValence = valence;
        if (maxValence < 0 || valence > maxValence) maxValence = valence;

        valenceSum += valence;
    }

    double meanValence = valenceSum / m.size_of_vertices();

    //check nonplanar faces
    int numNonpalanarFaces = 0;
    double degreeSum = 0;
    int minDegree = -1;
    int maxDegree = -1;

    for (Polyhedron::Face_const_iterator faceIter = m.facets_begin(); faceIter != m.facets_end(); ++faceIter) {

        //get degree of face
        int degree = faceIter->facet_degree();
        //face can be only nonplar if degree more than three
        if (degree >= 4) {

            //use circulator to go through all half facets
            Polyhedron::Facet::Halfedge_around_facet_const_circulator halfEdgeCir = faceIter->facet_begin();

            //get first vertex of face
            const Point& v0 = halfEdgeCir->vertex()->point();
            ++halfEdgeCir;

            //get second vertex of face
            const Point& v1 = halfEdgeCir->vertex()->point();
            ++halfEdgeCir;

            //get third vertex of face
            const Point& v2 = halfEdgeCir->vertex()->point();
            ++halfEdgeCir;

            //check if each vertex is coplanar with first three verticies
            for (int i = 3; i < degree; ++i) {

                //get next vertex of face
                const Point& v = halfEdgeCir->vertex()->point();
                ++halfEdgeCir;

                if (!CGAL::coplanar(v0, v1, v2, v)) {
                    numNonpalanarFaces++;

                    cout << "nonplanar face detected: \n";
                    cout << "(" << v0.x() << ", " << v0.y() << ", " << v0.z() << ")\n";
                    cout << "(" << v1.x() << ", " << v1.y() << ", " << v1.z() << ")\n";
                    cout << "(" << v2.x() << ", " << v2.y() << ", " << v2.z() << ")\n";
                    cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")\n";
                    break;
                }
            }
        }

        //update min degree
        if (minDegree < 0 || degree < minDegree) minDegree = degree;
        //update max degree
        if (maxDegree < 0 || degree > maxDegree) maxDegree = degree;

        degreeSum += degree;
    }

    double meanDegree = degreeSum / m.size_of_facets();

    //normalize border  to use sizeofborderedges
    m.normalize_border();

    //info
    cout
        << "Mesh Type: " << meshType << "\n"
        << "Number Vertices: " << m.size_of_vertices() << "\n"
        << "Number Edges: " << m.size_of_halfedges() / 2 << "\n"
        << "Number of Border Edges: " << m.size_of_border_edges() << "\n"
        << "Number of facets: " << m.size_of_facets() << "\n"
        << "Number of halfEdges: " << m.size_of_halfedges() << "\n"
        << "Mean vertex Valence: " << meanValence << "\n"
        << "Min vertex Valence: " << minValence << "\n"
        << "Max vertex Valence: " << maxValence << "\n"
        << "Mean Facet Degree: " << meanDegree << "\n"
        << "Min Facet Degree: " << minDegree << "\n"
        << "Max Facet Degree: " << maxDegree << "\n"
        << "Number Nonplanar Facets: " << numNonpalanarFaces << "\n";


}