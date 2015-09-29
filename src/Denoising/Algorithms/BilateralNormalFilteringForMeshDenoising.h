#ifndef BILATERALNORMALFILTERINGFORMESHDENOISING_H
#define BILATERALNORMALFILTERINGFORMESHDENOISING_H

/*
 @brief: "Bilateral Normal Filtering for Mesh Denoising" class
 @reference: Bilateral Normal Filtering for Mesh Denoising, TVCG 2011
*/

#include "MeshDenoisingBase.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"

class BilateralNormalFilteringForMeshDenoising : public MeshDenoisingBase
{
public:
    BilateralNormalFilteringForMeshDenoising(DataManager *_data_manager, ParameterSet *_parameter_set);
    ~BilateralNormalFilteringForMeshDenoising() {}

private:
    void denoise();
    void initParameters();

    double getSigmaC(TriMesh &mesh, std::vector<TriMesh::Point> &face_centroid, double multiple_sigma_c);
    void updateFilteredNormals(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals);
    void updateFilteredNormalsLocalScheme(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals);
    void updateFilteredNormalsGlobalScheme(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals);
};

#endif // BILATERALNORMALFILTERINGFORMESHDENOISING_H
