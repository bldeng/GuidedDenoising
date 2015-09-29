#ifndef BILATERALMESHDENOISING_H
#define BILATERALMESHDENOISING_H

/*
 @brief: "Bilateral Mesh Denoising" class
 @reference: Bilateral Mesh Denoising, SIGGRAPH 2003
*/

#include "MeshDenoisingBase.h"
#include <vector>
#include <queue>

class BilateralMeshDenoising : public MeshDenoisingBase
{
public:
    BilateralMeshDenoising(DataManager *_data_manager, ParameterSet *_parameter_set);
    ~BilateralMeshDenoising() {}

private:
    void denoise();
    void initParameters();

    void getAdaptiveVertexNeighbor(TriMesh &mesh, TriMesh::VertexHandle vh, double sigma_c,
                                   std::vector<TriMesh::VertexHandle> &vertex_neighbor);
};

#endif // BILATERALMESHDENOISING_H
