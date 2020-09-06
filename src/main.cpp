#include "MyMesh.h"
#include "IO.h"



int main(int argc, char* argv[])
{
    Polyhedron m ;
    SurfaceMesh surface;
    import_OFF_file(m, surface, argv[1]);
    //import_OBJ_file(m, "plane.obj");
    if (m.is_empty() && surface.is_empty()) return 1;
   
    MyMesh::print_mesh_info(m);
    
    
    //load veticies and colkor dtaa to vectors
    //std::vector<Point> verts;
    //std::vector<CGAL::Color> cols;
    //MyMesh::load_verticies_and_color(surface, verts, cols);

    //MyMesh::color_surface(surface, 0, 255, 0, 255);

    /////////////////////----------------------------------------------------------------------------------------

    MyMesh::segment_mesh(surface);


    //write_PLY(argv[2], surface);
    //write_OFF(argv[2], surface);

    return EXIT_SUCCESS;
}