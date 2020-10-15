#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <tuple>
#include "CGAL_includes.h"
using namespace std;

// Define how a color should be stored
namespace CGAL {
    template< class F >
    struct Output_rep< ::MyColor, F > {
        const ::MyColor& c;
        static const bool is_specialized = true;
        Output_rep(const ::MyColor& c) : c(c)
        { }
        std::ostream& operator() (std::ostream& out) const
        {
            if (is_ascii(out))
                out << int(c[0]) << " " << int(c[1]) << " " << int(c[2]) << " " << int(c[3]);
            else
                out.write(reinterpret_cast<const char*>(&c), sizeof(c));
            return out;
        }
    };
}

namespace IO {
    void import_OFF_file(Polyhedron& m, SurfaceMesh& surface, std::string filename);

    void import_OBJ_file(Polyhedron& m, std::string filename);

    //not FACES
    void write_PLY(std::string filename, SurfaceMesh& surface);

    void write_PLY(std::string filename, std::vector< Kernel::Point_3> points);

    //save path
    void write_PLY(std::string filename, vector< Eigen::Vector3f> path);

    void write_OFF(std::string filename, SurfaceMesh& surface);

}

