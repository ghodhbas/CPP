#include <iostream>
#include "MyMesh.h"

using namespace std;

int main()
{
    Polyhedron m ;
    MyMesh::import_OFF_file(m, "plane.off");
    //MyMesh::import_OBJ_file(m, "plane.obj");
    if (m.is_empty()) return 1;
   
    MyMesh::print_mesh_info(m);
    
    return EXIT_SUCCESS;
}