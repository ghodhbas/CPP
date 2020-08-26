//********************************************************** USING CGAL *******************************************************************************/
#include <iostream>
#include <fstream>
#include "MyMesh.h"

using namespace std;

int main()
{
   Polyhedron m;
   std::ifstream file("Meshes/plane.off");
    if (!(file >> m)) {
        cerr << "cannot read mesh";
        return 1;
    }

    print_mesh_info(m);
    
    return EXIT_SUCCESS;
}