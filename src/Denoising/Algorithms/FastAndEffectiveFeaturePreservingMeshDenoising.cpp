#include "FastAndEffectiveFeaturePreservingMeshDenoising.h"

FastAndEffectiveFeaturePreservingMeshDenoising::FastAndEffectiveFeaturePreservingMeshDenoising(DataManager *_data_manager, ParameterSet *_parameter_set)
    : MeshDenoisingBase(_data_manager, _parameter_set)
{
    initParameters();
}

void FastAndEffectiveFeaturePreservingMeshDenoising::denoise()
{
    // get data
    TriMesh mesh = data_manager_->getNoisyMesh();

    if(mesh.n_vertices() == 0)
        return;

    // update face normal
    std::vector<TriMesh::Normal> filtered_normals;
    updateFilteredNormals(mesh, filtered_normals);

    // get parameter for vertex update
    int vertex_iteration_number;
    if(!parameter_set_->getValue(QString("Vertex Iteration Num."), vertex_iteration_number))
        return;

    // update vertex position
    updateVertexPosition(mesh, filtered_normals, vertex_iteration_number, true);

    // update data
    data_manager_->setMesh(mesh);
    data_manager_->setDenoisedMesh(mesh);
}

void FastAndEffectiveFeaturePreservingMeshDenoising::initParameters()
{
    parameter_set_->removeAllParameter();

    QStringList strList_FaceNeighborType;
    strList_FaceNeighborType.push_back(QString("common vertices"));
    strList_FaceNeighborType.push_back(QString("common edges"));

    parameter_set_->addParameter(QString("Face Neighbor"), strList_FaceNeighborType, 0, QString("Face Neighbor"), QString("The type of the neighbor of the face."));

    parameter_set_->addParameter(QString("Threshold T"), 0.5, QString("Threshold T"), QString("Threshold for controlling the averaging weights."),
                                 true, 1.0e-10, 1.0);

    parameter_set_->addParameter(QString("Normal Iteration Num."), 20, QString("Normal Iteration Num."), QString("The iteration number for filtering face normal."),
                                 true, 1, 500);

    parameter_set_->addParameter(QString("Vertex Iteration Num."), 50, QString("Vertex Iteration Num."), QString("The iteration number for filtering vertex position."),
                                 true, 1, 500);

    parameter_set_->setName(QString("Fast and Effective Feature-Preserving Mesh Denoising"));
    parameter_set_->setLabel(QString("Fast and Effective Feature-Preserving Mesh Denoising"));
    parameter_set_->setIntroduction(QString("Fast and Effective Feature-Preserving Mesh Denoising -- Parameters"));
}

void FastAndEffectiveFeaturePreservingMeshDenoising::updateFilteredNormals(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
{
    // get parameter for normal update
    int face_neighbor_index;
    if(!parameter_set_->getStringListIndex(QString("Face Neighbor"), face_neighbor_index))
        return;
    double threshold_T;
    if(!parameter_set_->getValue(QString("Threshold T"), threshold_T))
        return;
    int normal_iteration_number;
    if(!parameter_set_->getValue(QString("Normal Iteration Num."), normal_iteration_number))
        return;

    FaceNeighborType face_neighbor_type = (face_neighbor_index == 0) ? kVertexBased : kEdgeBased;

    std::vector< std::vector<TriMesh::FaceHandle> > all_face_neighbor;
    getAllFaceNeighbor(mesh, all_face_neighbor, face_neighbor_type, true);

    filtered_normals.resize(mesh.n_faces());

    std::vector<TriMesh::Normal> previous_normals;
    getFaceNormal(mesh, previous_normals);

    for(int iter = 0; iter < normal_iteration_number; iter++)
    {
        for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
        {
            TriMesh::Normal ni = previous_normals[f_it->idx()];
            std::vector<TriMesh::FaceHandle> face_neighbor = all_face_neighbor[f_it->idx()];
            TriMesh::Normal temp_normal(0.0, 0.0, 0.0);
            for(int i = 0; i <(int)face_neighbor.size(); i++)
            {
                TriMesh::Normal nj = previous_normals[face_neighbor[i].idx()];
                double value = (ni | nj) - threshold_T;
                double weight = (value > 0) ? value * value : 0;
                temp_normal += nj * weight;
            }
            temp_normal.normalize();
            filtered_normals[f_it->idx()] = temp_normal;
        }
        previous_normals = filtered_normals;
    }
}
