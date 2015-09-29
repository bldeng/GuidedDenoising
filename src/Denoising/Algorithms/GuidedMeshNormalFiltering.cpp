#include "GuidedMeshNormalFiltering.h"
#include <ctime>
#include <iostream>

GuidedMeshNormalFiltering::GuidedMeshNormalFiltering(DataManager *_data_manager, ParameterSet *_parameter_set)
    : MeshDenoisingBase(_data_manager, _parameter_set)
{
    initParameters();
}

void GuidedMeshNormalFiltering::denoise()
{
    // get data
    TriMesh mesh = data_manager_->getNoisyMesh();

    if(mesh.n_vertices() == 0)
        return;

    // update face normal
    std::vector<TriMesh::Normal> filtered_normals;
    updateFilteredNormals(mesh, filtered_normals);

    // update data
    data_manager_->setMesh(mesh);
    data_manager_->setDenoisedMesh(mesh);
}

void GuidedMeshNormalFiltering::initParameters()
{
    parameter_set_->removeAllParameter();
    QStringList strList_DenoiseType;
    strList_DenoiseType.push_back(QString("Local"));
    strList_DenoiseType.push_back(QString("Global"));

    parameter_set_->addParameter(QString("Denoise Type"), strList_DenoiseType, 0, QString("Denoise Type"), QString("The type of denoise method."));

    QStringList strList_FaceNeighborType;
    strList_FaceNeighborType.push_back(QString("geometrical"));
    strList_FaceNeighborType.push_back(QString("topological"));

    parameter_set_->addParameter(QString("Face Neighbor"), strList_FaceNeighborType, 0, QString("Face Neighbor"), QString("The type of the neighbor of the face."));
    parameter_set_->addParameter(QString("include central face"), true, QString("include central face"), QString("Include the central face of the neighbor or not."));

    parameter_set_->addParameter(QString("Multiple(* avg face dis.)"), 2.0, QString("Multiple(* avg face dis.)"), QString("Radius for search geometrical neighbor of the face."),
                                 true, 0.1, 10.0);
    parameter_set_->addParameter(QString("Multiple(* sigma_s)"), 1.0, QString("Multiple(* sigma_s)"), QString("Standard deviation of spatial weight."),
                                 true, 1.0e-9, 10.0);

    parameter_set_->addParameter(QString("sigma_r"), 0.35, QString("sigma_r"), QString("Standard deviation of range weight."),
                                 true, 1.0e-9, 10.0);

    parameter_set_->addParameter(QString("(Local)Normal Iteration Num."), 20, QString("(Local)Normal Iteration Num."), QString("The iteration number of local scheme for filtering face normal."),
                                 true, 1, 500);
    parameter_set_->addParameter(QString("(Global)Normal Iteration Num."), 1, QString("(Global)Normal Iteration Num."), QString("The iteration number of global scheme for filtering face normal."),
                                 true, 1, 50);

    parameter_set_->addParameter(QString("smoothness"), 0.01, QString("smoothness"), QString("Smoothness parameter for global filtering."),
                                 true, 1.0e-9, 1.0);

    parameter_set_->addParameter(QString("Vertex Iteration Num."), 10, QString("Vertex Iteration Num."), QString("The iteration number for filtering vertex position."),
                                 true, 1, 500);


    parameter_set_->setName(QString("Guided Mesh Normal Filtering"));
    parameter_set_->setLabel(QString("Guided Mesh Normal Filtering"));
    parameter_set_->setIntroduction(QString("Guided Mesh Normal Filtering -- Parameters"));
}

void GuidedMeshNormalFiltering::getVertexBasedFaceNeighbor(TriMesh &mesh, TriMesh::FaceHandle fh, std::vector<TriMesh::FaceHandle> &face_neighbor)
{
    getFaceNeighbor(mesh, fh, kVertexBased, face_neighbor);
}

void GuidedMeshNormalFiltering::getRadiusBasedFaceNeighbor(TriMesh &mesh, TriMesh::FaceHandle fh, double radius, std::vector<TriMesh::FaceHandle> &face_neighbor)
{
    TriMesh::Point ci = mesh.calc_face_centroid(fh);
    std::vector<bool> flag((int)mesh.n_faces(), false);

    face_neighbor.clear();
    flag[fh.idx()] = true;
    std::queue<TriMesh::FaceHandle> queue_face_handle;
    queue_face_handle.push(fh);

    std::vector<TriMesh::FaceHandle> temp_face_neighbor;
    while(!queue_face_handle.empty())
    {
        TriMesh::FaceHandle temp_face_handle_queue = queue_face_handle.front();
        if(temp_face_handle_queue != fh)
            face_neighbor.push_back(temp_face_handle_queue);
        queue_face_handle.pop();
        getVertexBasedFaceNeighbor(mesh, temp_face_handle_queue, temp_face_neighbor);
        for(int i = 0; i < (int)temp_face_neighbor.size(); i++)
        {
            TriMesh::FaceHandle temp_face_handle = temp_face_neighbor[i];
            if(!flag[temp_face_handle.idx()])
            {
                TriMesh::Point cj = mesh.calc_face_centroid(temp_face_handle);
                double distance = (ci - cj).length();
                if(distance <= radius)
                    queue_face_handle.push(temp_face_handle);
                flag[temp_face_handle.idx()] = true;
            }
        }
    }
}

void GuidedMeshNormalFiltering::getAllFaceNeighborGMNF(TriMesh &mesh, MeshDenoisingBase::FaceNeighborType face_neighbor_type, double radius, bool include_central_face,
                                                       std::vector<std::vector<TriMesh::FaceHandle> > &all_face_neighbor)
{
    std::vector<TriMesh::FaceHandle> face_neighbor;
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        if(face_neighbor_type == kVertexBased)
            getVertexBasedFaceNeighbor(mesh, *f_it, face_neighbor);
        else if(face_neighbor_type == kRadiusBased)
            getRadiusBasedFaceNeighbor(mesh, *f_it, radius, face_neighbor);

        if(include_central_face)
            face_neighbor.push_back(*f_it);
        all_face_neighbor[f_it->idx()] = face_neighbor;
    }
}

void GuidedMeshNormalFiltering::getAllGuidedNeighborGMNF(TriMesh &mesh, std::vector<std::vector<TriMesh::FaceHandle> > &all_guided_neighbor)
{
    std::vector<TriMesh::FaceHandle> face_neighbor;
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        getVertexBasedFaceNeighbor(mesh, *f_it, face_neighbor);
        face_neighbor.push_back(*f_it);
        all_guided_neighbor[f_it->idx()] = face_neighbor;
    }
}

double GuidedMeshNormalFiltering::GaussianWeight(double distance, double sigma)
{
    return std::exp( -0.5 * distance * distance / (sigma * sigma));
}

double GuidedMeshNormalFiltering::NormalDistance(const TriMesh::Normal &n1, const TriMesh::Normal &n2)
{
    return (n1 - n2).length();
}

void GuidedMeshNormalFiltering::getFaceNeighborInnerEdge(TriMesh &mesh, std::vector<TriMesh::FaceHandle> &face_neighbor, std::vector<TriMesh::EdgeHandle> &inner_edge)
{
    inner_edge.clear();
    std::vector<bool> edge_flag((int)mesh.n_edges(), false);
    std::vector<bool> face_flag((int)mesh.n_faces(), false);

    for(int i = 0; i < (int)face_neighbor.size(); i++)
        face_flag[face_neighbor[i].idx()] = true;

    for(int i = 0; i < (int)face_neighbor.size(); i++)
    {
        for(TriMesh::FaceEdgeIter fe_it = mesh.fe_iter(face_neighbor[i]); fe_it.is_valid(); fe_it++)
        {
            if((!edge_flag[fe_it->idx()]) && (!mesh.is_boundary(*fe_it)))
            {
                edge_flag[fe_it->idx()] = true;
                TriMesh::HalfedgeHandle heh = mesh.halfedge_handle(*fe_it, 0);
                TriMesh::FaceHandle f = mesh.face_handle(heh);
                TriMesh::HalfedgeHandle heho = mesh.opposite_halfedge_handle(heh);
                TriMesh::FaceHandle fo = mesh.face_handle(heho);
                if(face_flag[f.idx()] && face_flag[fo.idx()])
                    inner_edge.push_back(*fe_it);
            }
        }
    }
}

void GuidedMeshNormalFiltering::getRangeAndMeanNormal(TriMesh &mesh, std::vector<std::vector<TriMesh::FaceHandle> > &all_guided_neighbor,
                                                       std::vector<double> &face_area, std::vector<TriMesh::Normal> &normals,
                                                       std::vector<std::pair<double, TriMesh::Normal> > &range_and_mean_normal)
{
    const double epsilon = 1.0e-9;

    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        int index = f_it->idx();
        std::vector<TriMesh::FaceHandle> face_neighbor = all_guided_neighbor[index];

        double metric = 0.0;
        TriMesh::Normal average_normal(0.0, 0.0, 0.0);
        double maxdiff = -1.0;

        for(int i = 0; i < (int)face_neighbor.size(); i++)
        {
            int index_i = face_neighbor[i].idx();
            double area_weight = face_area[index_i];
            TriMesh::Normal ni = normals[index_i];
            average_normal += ni * area_weight;

            for(int j = i+1; j < (int)face_neighbor.size(); j++)
            {
                int index_j = face_neighbor[j].idx();
                TriMesh::Normal nj = normals[index_j];
                double diff = NormalDistance(ni, nj);

                if(diff > maxdiff)
                {
                    maxdiff = diff;
                }
            }
        }

        std::vector<TriMesh::EdgeHandle> inner_edge_handle;
        getFaceNeighborInnerEdge(mesh, face_neighbor, inner_edge_handle);
        double sum_tv = 0.0, max_tv = -1.0;
        for(int i = 0; i < (int)inner_edge_handle.size(); i++)
        {
            TriMesh::HalfedgeHandle heh = mesh.halfedge_handle(inner_edge_handle[i], 0);
            TriMesh::FaceHandle f = mesh.face_handle(heh);
            TriMesh::Normal n1 = normals[f.idx()];
            TriMesh::HalfedgeHandle heho = mesh.opposite_halfedge_handle(heh);
            TriMesh::FaceHandle fo = mesh.face_handle(heho);
            TriMesh::Normal n2 = normals[fo.idx()];
            double current_tv = NormalDistance(n1, n2);
            max_tv = (current_tv > max_tv)? current_tv : max_tv;
            sum_tv += current_tv;
        }

        average_normal.normalize();
        metric = maxdiff * max_tv / (sum_tv + epsilon);

        range_and_mean_normal[index] = std::make_pair(metric, average_normal);
    }
}

void GuidedMeshNormalFiltering::getGuidedNormals(TriMesh &mesh, std::vector<std::vector<TriMesh::FaceHandle> > &all_guided_neighbor,
                                                 std::vector<double> &face_area, std::vector<TriMesh::Normal> &normals,
                                                 std::vector<std::pair<double, TriMesh::Normal> > range_and_mean_normal,
                                                 std::vector<TriMesh::Normal> &guided_normals)
{
    getRangeAndMeanNormal(mesh, all_guided_neighbor, face_area, normals, range_and_mean_normal);

    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        std::vector<TriMesh::FaceHandle> face_neighbor = all_guided_neighbor[f_it->idx()];
        double min_range = 1.0e8;
        int min_idx = 0;
        for(int i = 0; i < (int)face_neighbor.size(); i++)
        {
            double current_range = range_and_mean_normal[face_neighbor[i].idx()].first;
            if(min_range > current_range){
                min_range = current_range;
                min_idx = i;
            }
        }
        TriMesh::FaceHandle min_face_handle = face_neighbor[min_idx];
        guided_normals[f_it->idx()] = range_and_mean_normal[min_face_handle.idx()].second;
    }
}

double GuidedMeshNormalFiltering::getRadius(double multiple, TriMesh &mesh)
{
    std::vector<TriMesh::Point> centroid;
    getFaceCentroid(mesh, centroid);

    double radius = 0.0;
    double num = 0.0;
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        TriMesh::Point fi = centroid[f_it->idx()];
        for(TriMesh::FaceFaceIter ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ff_it++)
        {
            TriMesh::Point fj = centroid[ff_it->idx()];
            radius += (fj - fi).length();
            num++;
        }
    }
    return radius * multiple / num;
}

double GuidedMeshNormalFiltering::getSigmaS(double multiple, std::vector<TriMesh::Point> &centroid, TriMesh &mesh)
{
    double sigma_s = 0.0, num = 0.0;
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        TriMesh::Point fi = centroid[f_it->idx()];
        for(TriMesh::FaceFaceIter ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ff_it++)
        {
            TriMesh::Point fj = centroid[ff_it->idx()];
            sigma_s += (fj - fi).length();
            num++;
        }
    }
    return sigma_s * multiple / num;
}

void GuidedMeshNormalFiltering::updateFilteredNormals(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
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

void GuidedMeshNormalFiltering::updateFilteredNormalsLocalScheme(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
{
    filtered_normals.resize((int)mesh.n_faces());
    // get parameter for local scheme normal update
    int face_neighbor_index;
    if(!parameter_set_->getStringListIndex(QString("Face Neighbor"), face_neighbor_index))
        return;
    bool include_central_face;
    if(!parameter_set_->getValue(QString("include central face"), include_central_face))
        return;
    double multiple_radius;
    if(!parameter_set_->getValue(QString("Multiple(* avg face dis.)"), multiple_radius))
        return;
    double multiple_sigma_s;
    if(!parameter_set_->getValue(QString("Multiple(* sigma_s)"), multiple_sigma_s))
        return;
    int normal_iteration_number;
    if(!parameter_set_->getValue(QString("(Local)Normal Iteration Num."), normal_iteration_number))
        return;
    double sigma_r;
    if(!parameter_set_->getValue(QString("sigma_r"), sigma_r))
        return;
    int vertex_iteration_number;
    if(!parameter_set_->getValue(QString("Vertex Iteration Num."), vertex_iteration_number))
        return;

    FaceNeighborType face_neighbor_type = face_neighbor_index == 0 ? kRadiusBased : kVertexBased;

    double radius;
    if(face_neighbor_type == kRadiusBased)
        radius = getRadius(multiple_radius, mesh);

    std::vector<std::vector<TriMesh::FaceHandle> > all_face_neighbor((int)mesh.n_faces());
    getAllFaceNeighborGMNF(mesh, face_neighbor_type, radius, include_central_face, all_face_neighbor);
    std::vector<std::vector<TriMesh::FaceHandle> > all_guided_neighbor((int)mesh.n_faces());
    getAllGuidedNeighborGMNF(mesh, all_guided_neighbor);
    getFaceNormal(mesh, filtered_normals);

    std::vector<double> face_area((int)mesh.n_faces());
    std::vector<TriMesh::Point> face_centroid((int)mesh.n_faces());
    std::vector<TriMesh::Normal> previous_normals((int)mesh.n_faces());
    std::vector<TriMesh::Normal> guided_normals((int)mesh.n_faces());
    std::vector<std::pair<double, TriMesh::Normal> > range_and_mean_normal((int)mesh.n_faces());
    for(int iter = 0; iter < normal_iteration_number; iter++)
    {
        getFaceCentroid(mesh, face_centroid);
        double sigma_s = getSigmaS(multiple_sigma_s, face_centroid, mesh);
        getFaceArea(mesh, face_area);
        getFaceNormal(mesh, previous_normals);

        getGuidedNormals(mesh, all_guided_neighbor, face_area, previous_normals, range_and_mean_normal, guided_normals);

        // Filtered Face Normals
        for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
        {
            int index = f_it->idx();
            const std::vector<TriMesh::FaceHandle> face_neighbor = all_face_neighbor[index];
            TriMesh::Normal filtered_normal(0.0, 0.0, 0.0);
            for(int j = 0; j < (int)face_neighbor.size(); j++)
            {
                int current_face_index = face_neighbor[j].idx();

                double spatial_dis = (face_centroid[index] - face_centroid[current_face_index]).length();
                double spatial_weight = GaussianWeight(spatial_dis, sigma_s);
                double range_dis = (guided_normals[index] - guided_normals[current_face_index]).length();
                double range_weight = GaussianWeight(range_dis, sigma_r);

                filtered_normal += previous_normals[current_face_index] * (face_area[current_face_index] * spatial_weight * range_weight);
            }
            if(face_neighbor.size())
                filtered_normals[index] = filtered_normal.normalize();
        }

        // immediate update vertex position
        updateVertexPosition(mesh, filtered_normals, vertex_iteration_number, false);
    }
}

void GuidedMeshNormalFiltering::updateFilteredNormalsGlobalScheme(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals)
{
    filtered_normals.resize((int)mesh.n_faces());

    // get parameter for local scheme normal update
    int face_neighbor_index;
    if(!parameter_set_->getStringListIndex(QString("Face Neighbor"), face_neighbor_index))
        return;
    bool include_central_face;
    if(!parameter_set_->getValue(QString("include central face"), include_central_face))
        return;
    double multiple_radius;
    if(!parameter_set_->getValue(QString("Multiple(* avg face dis.)"), multiple_radius))
        return;
    double multiple_sigma_s;
    if(!parameter_set_->getValue(QString("Multiple(* sigma_s)"), multiple_sigma_s))
        return;
    int normal_iteration_number;
    if(!parameter_set_->getValue(QString("(Global)Normal Iteration Num."), normal_iteration_number))
        return;
    double sigma_r;
    if(!parameter_set_->getValue(QString("sigma_r"), sigma_r))
        return;
    double smoothness;
    if(!parameter_set_->getValue(QString("smoothness"), smoothness))
        return;
    int vertex_iteration_number;
    if(!parameter_set_->getValue(QString("Vertex Iteration Num."), vertex_iteration_number))
        return;

    FaceNeighborType face_neighbor_type = face_neighbor_index == 0 ? kRadiusBased : kVertexBased;

    std::vector< Eigen::Triplet<double> > coef_triple;
    std::vector< Eigen::Triplet<double> > weight_triple;
    Eigen::SparseMatrix<double> weight_matrix((int)mesh.n_faces(), (int)mesh.n_faces());
    Eigen::SparseMatrix<double> normalize_matrix((int)mesh.n_faces(), (int)mesh.n_faces());
    Eigen::SparseMatrix<double> identity_matrix((int)mesh.n_faces(), (int)mesh.n_faces());
    identity_matrix.setIdentity();

    double radius;
    if(face_neighbor_type == kRadiusBased)
        radius = getRadius(multiple_radius, mesh);

    std::vector<std::vector<TriMesh::FaceHandle> > all_face_neighbor((int)mesh.n_faces());
    getAllFaceNeighborGMNF(mesh, face_neighbor_type, radius, include_central_face, all_face_neighbor);
    std::vector<std::vector<TriMesh::FaceHandle> > all_guided_neighbor((int)mesh.n_faces());
    getAllGuidedNeighborGMNF(mesh, all_guided_neighbor);
    getFaceNormal(mesh, filtered_normals);

    std::vector<double> face_area((int)mesh.n_faces());
    std::vector<TriMesh::Point> face_centroid((int)mesh.n_faces());
    std::vector<TriMesh::Normal> previous_normals((int)mesh.n_faces());
    std::vector<TriMesh::Normal> guided_normals((int)mesh.n_faces());
    std::vector<std::pair<double, TriMesh::Normal> > range_and_mean_normal((int)mesh.n_faces());
    for(int iter = 0; iter < normal_iteration_number; iter++)
    {
        getFaceCentroid(mesh, face_centroid);
        double sigma_s = getSigmaS(multiple_sigma_s, face_centroid, mesh);
        getFaceArea(mesh, face_area);
        getFaceNormal(mesh, previous_normals);

        getGuidedNormals(mesh, all_guided_neighbor, face_area, previous_normals, range_and_mean_normal, guided_normals);

        // construct matrix
        coef_triple.clear();
        weight_triple.clear();

        for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
        {
            int index_i = f_it->idx();
            TriMesh::Normal ni = guided_normals[index_i];
            TriMesh::Point ci = face_centroid[index_i];
            std::vector<TriMesh::FaceHandle> face_neighbor = all_face_neighbor[index_i];            
            double weight_sum = 0.0;
            for(int i = 0; i < (int)face_neighbor.size(); i++)
            {
                int index_j = face_neighbor[i].idx();
                TriMesh::Normal nj = guided_normals[index_j];
                TriMesh::Point cj = face_centroid[index_j];
                double spatial_weight = GaussianWeight((ci - cj).length(), sigma_s);
                double range_weight = GaussianWeight((ni - nj).length(), sigma_r);
                double weight = face_area[index_j] * spatial_weight * range_weight;
                coef_triple.push_back( Eigen::Triplet<double>(index_i, index_j, weight) );
                weight_sum += weight;
            }
            weight_triple.push_back( Eigen::Triplet<double>(index_i, index_i , 1.0 / weight_sum) );
        }
        weight_matrix.setFromTriplets(coef_triple.begin(), coef_triple.end());
        normalize_matrix.setFromTriplets(weight_triple.begin(), weight_triple.end());
        Eigen::SparseMatrix<double> matrix = identity_matrix - normalize_matrix * weight_matrix;
        Eigen::SparseMatrix<double> coef_matrix = (1 - smoothness) * matrix.transpose() * matrix + smoothness * identity_matrix;

        // construct right term
        Eigen::MatrixXd right_term((int)mesh.n_faces(), 3);
        for(int i = 0; i < (int)previous_normals.size(); i++)
        {
            right_term(i, 0) = previous_normals[i][0];
            right_term(i, 1) = previous_normals[i][1];
            right_term(i, 2) = previous_normals[i][2];
        }
        right_term = smoothness * right_term;

        // solve Ax = b
        Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
        solver.analyzePattern(coef_matrix);
        solver.factorize(coef_matrix);
        Eigen::MatrixX3d filtered_normals_matrix = solver.solve(right_term);
        filtered_normals_matrix.rowwise().normalize();
        for(int i = 0; i < (int)filtered_normals.size(); i++)
        {
            filtered_normals[i][0] = filtered_normals_matrix(i, 0);
            filtered_normals[i][1] = filtered_normals_matrix(i, 1);
            filtered_normals[i][2] = filtered_normals_matrix(i, 2);
        }
        // immediate update vertex position
        updateVertexPosition(mesh, filtered_normals, vertex_iteration_number, false);
    }
}
