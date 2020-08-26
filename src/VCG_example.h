#pragma once
//********************************************************** USING VCG *******************************************************************************/

#include <iostream>
#include <MyMesh.h>

#include <vcg/complex/complex.h>
#include<wrap/io_trimesh/import_off.h>
using namespace vcg;
using namespace std;

class MyEdge;
class MyFace;
class MyVertex;

struct MyUsedTypes : public UsedTypes<  Use<MyVertex>   ::AsVertexType,
    Use<MyEdge>     ::AsEdgeType,
    Use<MyFace>     ::AsFaceType> {};

class MyVertex : public Vertex<MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags  > {};
class MyFace : public Face< MyUsedTypes, face::FFAdj, face::VertexRef, face::BitFlags > {};
class MyEdge : public Edge<MyUsedTypes> {};
class MyMesh : public tri::TriMesh< vector<MyVertex>, vector<MyFace>, vector<MyEdge>  > {};


int main(int argc, char** argv) {
   
    if (argc < 2)
    {
        printf("Usage trimesh_base <meshfilename.obj>\n");
        return -1;
    }

    MyMesh m;
    if (tri::io::ImporterOFF<MyMesh>::Open(m, argv[1]) != 0)
    {
      printf("Error reading file  %s\n", argv[1]);
      exit(0);
    }

    //update normals
    tri::UpdateNormal<MyMesh>::PerVertexClear(m);
    tri::UpdateNormal<MyMesh>::PerVertex(m);
    printf("Input mesh  vn:%i fn:%i\n", m.VN(), m.FN());


    return 0;
}