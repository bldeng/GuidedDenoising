#ifndef NONITERATIVEFEATUREPRESERVINGMESHFILTERING_H
#define NONITERATIVEFEATUREPRESERVINGMESHFILTERING_H

/*
 @brief: "Non-Iterative, Feature-Preserving Mesh Smoothing" class
 @reference: Non-Iterative, Feature-Preserving Mesh Smoothing, SIGGRAPH 2003
*/

#include "MeshDenoisingBase.h"
#include <vector>
#include <queue>
#include <set>

class NonIterativeFeaturePreservingMeshFiltering : public MeshDenoisingBase
{
public:
    NonIterativeFeaturePreservingMeshFiltering(DataManager *_data_manager, ParameterSet *_parameter_set);
    ~NonIterativeFeaturePreservingMeshFiltering() {}

private:
    void denoise();
    void initParameters();

    void getVertexFaceNeighbor(TriMesh &mesh, std::vector<TriMesh::Point> &face_centroid, TriMesh::VertexHandle vertex_handle, double r,
                               std::vector<TriMesh::FaceHandle> &vertex_face_neighbor);
    void mollifiedNormals(TriMesh &mesh, std::vector<TriMesh::Point> &face_centroid, std::vector<double> &face_area, double sigma_f,
                          std::vector<TriMesh::Normal> &mollified_normals);
    TriMesh::Point projectPoint(TriMesh::Point pt, TriMesh::Point centroid, TriMesh::Normal n);
};

#endif // NONITERATIVEFEATUREPRESERVINGMESHFILTERING_H
