#ifndef FASTANDEFFECTIVEFEATUREPRESERVINGMESHDENOISING_H
#define FASTANDEFFECTIVEFEATUREPRESERVINGMESHDENOISING_H

/*
 @brief: "Fast and Effective Feature Preserving Mesh Denoising" class
 @reference: Fast and Effective Feature Preserving Mesh Denoising, TVCG 2007
*/

#include "MeshDenoisingBase.h"
#include <vector>
#include <set>

class FastAndEffectiveFeaturePreservingMeshDenoising : public MeshDenoisingBase
{
public:
    FastAndEffectiveFeaturePreservingMeshDenoising(DataManager *_data_manager, ParameterSet *_parameter_set);
    ~FastAndEffectiveFeaturePreservingMeshDenoising() {}

private:
    void denoise();
    void initParameters();

    void updateFilteredNormals(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals);
};

#endif // FASTANDEFFECTIVEFEATUREPRESERVINGMESHDENOISING_H
