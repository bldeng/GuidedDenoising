#include "BilateralNormalFilteringForMeshDenoising.h"

BilateralNormalFilteringForMeshDenoising::BilateralNormalFilteringForMeshDenoising(DataManager *_data_manager, ParameterSet *_parameter_set)
    : MeshDenoisingBase(_data_manager, _parameter_set)
{
    initParameters();
}

void BilateralNormalFilteringForMeshDenoising::denoise()
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

void BilateralNormalFilteringForMeshDenoising::initParameters()
{
    parameter_set_->removeAllParameter();

    QStringList strList_DenoiseType;
    strList_DenoiseType.push_back(QString("Local"));
    strList_DenoiseType.push_back(QString("Global"));

    parameter_set_->addParameter(QString("Denoise Type"), strList_DenoiseType, 0, QString("Denoise Type"), QString("The type of denoise method."));

    QStringList strList_FaceNeighborType;
    strList_FaceNeighborType.push_back(QString("common vertices"));
    strList_FaceNeighborType.push_back(QString("common edges"));

    parameter_set_->addParameter(QString("Face Neighbor"), strList_FaceNeighborType, 0, QString("Face Neighbor"), QString("The type of the neighbor of the face."));

    parameter_set_->addParameter(QString("Multiple(* sigma_c)"), 1.0, QString("Multiple(* sigma_c)"), QString("Standard deviation of spatial weight."),
                                 true, 1.0e-9, 10.0);

    parameter_set_->addParameter(QString("sigma_s"), 0.35, QString("sigma_s"), QString("Standard deviation of range weight."),
                                 true, 1.0e-9, 10.0);

    parameter_set_->addParameter(QString("Normal Iteration Num."), 20, QString("Normal Iteration Num."), QString("The iteration number for filtering face normal."),
                                 true, 1, 500);

    parameter_set_->addParameter(QString("Smoothness"), 0.01, QString("Smoothness"), QString("Smoothness parameter for global filtering."),
                                 true, 1.0e-9, 1.0);

    parameter_set_->addParameter(QString("Vertex Iteration Num."), 10, QString("Vertex Iteration Num."), QString("The iteration number for filtering vertex position."),
                                 true, 1, 500);

    parameter_set_->setName(QString("Bilateral Normal Filtering for Mesh Denoising"));
    parameter_set_->setLabel(QString("Bilateral Normal Filtering for Mesh Denoising"));
    parameter_set_->setIntroduction(QString("Bilateral Normal Filtering for Mesh Denoising -- Parameters"));
}

double BilateralNormalFilteringForMeshDenoising::getSigmaC(TriMesh &mesh, std::vector<TriMesh::Point> &face_centroid, double multiple_sigma_c)
{
    double sigma_c = 0.0;
    double num = 0.0;
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        TriMesh::Point ci = face_centroid[f_it->idx()];
        for(TriMesh::FaceFaceIter ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ff_it++)
        {
            TriMesh::Point cj = face_centroid[ff_it->idx()];
            sigma_c += (ci - cj).length();
            num++;
        }
    }
    sigma_c *= multiple_sigma_c / num;

    return sigma_c;
}

void BilateralNormalFilteringForMeshDenoising::updateFilteredNormals(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
{
    // get parameter for normal update
    int denoise_index;
    if(!parameter_set_->getStringListIndex(QString("Denoise Type"), denoise_index))
        return;

    DenoiseType denoise_type = (denoise_index == 0) ? kLocal : kGlobal;

    if(denoise_type == kLocal)
        updateFilteredNormalsLocalScheme(mesh, filtered_normals);
    else if(denoise_type == kGlobal)
        updateFilteredNormalsGlobalScheme(mesh, filtered_normals);
}

void BilateralNormalFilteringForMeshDenoising::updateFilteredNormalsLocalScheme(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
{
    filtered_normals.resize(mesh.n_faces());

    // get parameter for local scheme normal update
    int face_neighbor_index;
    if(!parameter_set_->getStringListIndex(QString("Face Neighbor"), face_neighbor_index))
        return;
    double multiple_sigma_c;
    if(!parameter_set_->getValue(QString("Multiple(* sigma_c)"), multiple_sigma_c))
        return;
    int normal_iteration_number;
    if(!parameter_set_->getValue(QString("Normal Iteration Num."), normal_iteration_number))
        return;
    double sigma_s;
    if(!parameter_set_->getValue(QString("sigma_s"), sigma_s))
        return;

    FaceNeighborType face_neighbor_type = (face_neighbor_index == 0) ? kVertexBased : kEdgeBased;

    std::vector< std::vector<TriMesh::FaceHandle> > all_face_neighbor;
    getAllFaceNeighbor(mesh, all_face_neighbor, face_neighbor_type, false);
    std::vector<TriMesh::Normal> previous_normals;
    getFaceNormal(mesh, previous_normals);
    std::vector<double> face_area;
    getFaceArea(mesh, face_area);
    std::vector<TriMesh::Point> face_centroid;
    getFaceCentroid(mesh, face_centroid);

    double sigma_c = getSigmaC(mesh, face_centroid, multiple_sigma_c);

    for(int iter = 0; iter < normal_iteration_number; iter++)
    {
        for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
        {
            int index_i = f_it->idx();
            TriMesh::Normal ni = previous_normals[index_i];
            TriMesh::Point ci = face_centroid[index_i];
            std::vector<TriMesh::FaceHandle> face_neighbor = all_face_neighbor[index_i];
            int size = (int)face_neighbor.size();
            TriMesh::Normal temp_normal(0.0, 0.0, 0.0);
            double weight_sum = 0.0;
            for(int i = 0; i < size; i++)
            {
                int index_j = face_neighbor[i].idx();
                TriMesh::Normal nj = previous_normals[index_j];
                TriMesh::Point cj = face_centroid[index_j];

                double spatial_distance = (ci - cj).length();
                double spatial_weight = std::exp( -0.5 * spatial_distance * spatial_distance / (sigma_c * sigma_c) );
                double range_distance = (ni - nj).length();
                double range_weight = std::exp( -0.5 * range_distance * range_distance / (sigma_s * sigma_s) );

                double weight = face_area[index_j] * spatial_weight * range_weight;
                weight_sum += weight;
                temp_normal += nj * weight;
            }
            temp_normal /= weight_sum;
            temp_normal.normalize();
            filtered_normals[index_i] = temp_normal;
        }
        previous_normals = filtered_normals;
    }
}

void BilateralNormalFilteringForMeshDenoising::updateFilteredNormalsGlobalScheme(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
{
    filtered_normals.resize(mesh.n_faces());

    // get parameter for local scheme normal update
    int face_neighbor_index;
    if(!parameter_set_->getStringListIndex(QString("Face Neighbor"), face_neighbor_index))
        return;
    double multiple_sigma_c;
    if(!parameter_set_->getValue(QString("Multiple(* sigma_c)"), multiple_sigma_c))
        return;
    double smoothness;
    if(!parameter_set_->getValue(QString("Smoothness"), smoothness))
        return;
    double sigma_s;
    if(!parameter_set_->getValue(QString("sigma_s"), sigma_s))
        return;

    FaceNeighborType face_neighbor_type = (face_neighbor_index == 0) ? kVertexBased : kEdgeBased;

    std::vector<TriMesh::Normal> initial_normals;
    getFaceNormal(mesh, initial_normals);
    std::vector<double> face_area;
    getFaceArea(mesh, face_area);
    std::vector<TriMesh::Point> face_centroid;
    getFaceCentroid(mesh, face_centroid);

    double sigma_c = getSigmaC(mesh, face_centroid, multiple_sigma_c);

    // construct matrix
    std::vector< Eigen::Triplet<double> > coeff_triple; coeff_triple.clear();
    std::vector< Eigen::Triplet<double> > weight_triple; weight_triple.clear();

    Eigen::SparseMatrix<double> weight_matrix((int)mesh.n_faces(), (int)mesh.n_faces());
    Eigen::SparseMatrix<double> normalize_matrix((int)mesh.n_faces(), (int)mesh.n_faces());
    Eigen::SparseMatrix<double> identity_matrix((int)mesh.n_faces(), (int)mesh.n_faces());
    identity_matrix.setIdentity();

    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        int index_i = f_it->idx();
        TriMesh::Normal ni = initial_normals[index_i];
        TriMesh::Point ci = face_centroid[index_i];
        std::vector<TriMesh::FaceHandle> face_neighbor;
        getFaceNeighbor(mesh, *f_it, face_neighbor_type, face_neighbor);
        double weight_sum = 0.0;
        for(int i = 0; i < (int)face_neighbor.size(); i++)
        {
            int index_j = face_neighbor[i].idx();
            TriMesh::Normal nj = initial_normals[index_j];
            TriMesh::Point cj = face_centroid[index_j];
            double spatial_distance = (ci - cj).length();
            double spatial_weight = std::exp( -0.5 * spatial_distance * spatial_distance / (sigma_c * sigma_c) );
            double range_distance = (ni - nj).length();
            double range_weight = std::exp( -0.5 * range_distance * range_distance / (sigma_s * sigma_s) );
            double weight = face_area[index_j] * spatial_weight * range_weight;
            coeff_triple.push_back( Eigen::Triplet<double>(index_i, index_j, weight) );
            weight_sum += weight;
        }
        if(weight_sum)
            weight_triple.push_back( Eigen::Triplet<double>(index_i, index_i , 1.0 / weight_sum) );
    }
    weight_matrix.setFromTriplets(coeff_triple.begin(), coeff_triple.end());
    normalize_matrix.setFromTriplets(weight_triple.begin(), weight_triple.end());
    Eigen::SparseMatrix<double> matrix = identity_matrix - normalize_matrix * weight_matrix;
    Eigen::SparseMatrix<double> coeff_matrix = (1 - smoothness) * matrix.transpose() * matrix + smoothness * identity_matrix;

    // construct right term
    Eigen::MatrixXd right_term(mesh.n_faces(), 3);
    for(int i = 0; i < (int)initial_normals.size(); i++)
    {
        right_term(i, 0) = initial_normals[i][0];
        right_term(i, 1) = initial_normals[i][1];
        right_term(i, 2) = initial_normals[i][2];
    }
    right_term = smoothness * right_term;

    // solve Ax = b
    Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
    solver.analyzePattern(coeff_matrix);
    solver.factorize(coeff_matrix);
    Eigen::MatrixX3d filtered_normals_matrix = solver.solve(right_term);
    filtered_normals_matrix.rowwise().normalize();
    for(int i = 0; i < (int)filtered_normals.size(); i++)
    {
        filtered_normals[i][0] = filtered_normals_matrix(i, 0);
        filtered_normals[i][1] = filtered_normals_matrix(i, 1);
        filtered_normals[i][2] = filtered_normals_matrix(i, 2);
    }
}
