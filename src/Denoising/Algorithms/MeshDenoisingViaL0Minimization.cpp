#include "MeshDenoisingViaL0Minimization.h"

MeshDenoisingViaL0Minimization::MeshDenoisingViaL0Minimization(DataManager *_data_manager, ParameterSet *_parameter_set)
    : MeshDenoisingBase(_data_manager, _parameter_set)
{
    initParameters();
}

void MeshDenoisingViaL0Minimization::denoise()
{
    // get data
    TriMesh mesh = data_manager_->getNoisyMesh();

    if(mesh.n_vertices() == 0)
        return;

    // get parameters
    double mu_beta, beta, beta_max;
    if(!parameter_set_->getValue(QString("mu_beta"), mu_beta))
        return;
    if(!parameter_set_->getValue(QString("beta"), beta))
        return;
    if(!parameter_set_->getValue(QString("beta_max"), beta_max))
        return;
    double mu_alpha, alpha;
    if(!parameter_set_->getValue(QString("mu_alpha"), mu_alpha))
        return;
    if(!parameter_set_->getValue(QString("alpha"), alpha))
        return;
    double lambda;
    if(!parameter_set_->getValue(QString("lambda"), lambda))
        return;

    // do optimization
    Eigen::MatrixXd initial_vertices_matrix;
    getInitialVerticesMatrix(mesh, initial_vertices_matrix);

    std::vector<TriMesh::Point> area_based_edge_operator;
    std::vector< std::vector<TriMesh::VertexHandle> > edge_vertex_handle;
    std::vector< std::vector<double> > coef;
    std::vector<TriMesh::Point> delta;

    while(beta < beta_max){
        calculateAreaBasedEdgeOperator(mesh, area_based_edge_operator, edge_vertex_handle, coef);
        // update delta
        solveDelta(area_based_edge_operator, lambda, beta, delta);
        // update vertices
        solveVertices(mesh, initial_vertices_matrix, edge_vertex_handle, coef, delta, alpha, beta);
        beta *= mu_beta;
        alpha *= mu_alpha;
    }

    // update data
    data_manager_->setMesh(mesh);
    data_manager_->setDenoisedMesh(mesh);
}

void MeshDenoisingViaL0Minimization::initParameters()
{
    parameter_set_->removeAllParameter();

    parameter_set_->addParameter(QString("mu_beta"), 1.414, QString("mu_beta"), QString("Update ratio for beta."),
                                 true, 1.001, 10000.0);
    parameter_set_->addParameter(QString("beta"), 0.001, QString("beta"), QString("Initial beta value in optimization."),
                                 true, 1.0e-9, 10000.0);
    parameter_set_->addParameter(QString("beta_max"), 1000.0, QString("beta_max"), QString("Max beta value in optimization."),
                                 true, 10.0, 1.0e9);
    parameter_set_->addParameter(QString("mu_alpha"), 0.5, QString("mu_alpha"), QString("Update ratio for alpha."),
                                 true, 1.0e-9, 0.9999);

    TriMesh mesh = data_manager_->getNoisyMesh();
    if(mesh.n_vertices() == 0)
    {
        parameter_set_->addParameter(QString("alpha"), 0.0001, QString("alpha"), QString("Initial alpha value in optimization."),
                                     true, 0.0, 1.0e9);
        parameter_set_->addParameter(QString("lambda"), 0.0001, QString("lambda"), QString("Lambda value in optimization."),
                                     true, 1.0e-9, 1.0e9);
    }
    else
    {
        double mean_edge_length = getAverageEdgeLength(mesh);
        double mean_dihedral_angle = getAverageDihedralAngle(mesh);
        parameter_set_->addParameter(QString("alpha"), 0.1 * mean_dihedral_angle, QString("alpha"), QString("Initial alpha value in optimization."),
                                     true, 0.0, 1.0e9);
        parameter_set_->addParameter(QString("lambda"), 0.2 * mean_edge_length * mean_edge_length * mean_dihedral_angle, QString("lambda"), QString("Lambda value in optimization."),
                                     true, 1.0e-9, 1.0e9);
    }

    parameter_set_->setName(QString("Mesh Denoising via L0 Minimization"));
    parameter_set_->setLabel(QString("Mesh Denoising via L0 Minimization"));
    parameter_set_->setIntroduction(QString("Mesh Denoising via L0 Minimization -- Parameters"));
}

double MeshDenoisingViaL0Minimization::getAverageDihedralAngle(TriMesh &mesh)
{
    mesh.request_face_normals();
    mesh.update_face_normals();

    double mean_dihedral_angle = 0.0;
    double num = 0.0;
    for(TriMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); e_it++)
    {
        if(!mesh.is_boundary(*e_it))
            mean_dihedral_angle += mesh.calc_dihedral_angle(*e_it);
        num++;
    }

    return mean_dihedral_angle / num;
}

void MeshDenoisingViaL0Minimization::calculateAreaBasedEdgeOperator(TriMesh &mesh,
                                                                    std::vector<TriMesh::Point> &area_based_edge_operator,
                                                                    std::vector< std::vector<TriMesh::VertexHandle> > &edge_vertex_handle,
                                                                    std::vector< std::vector<double> > &coef)
{
    std::vector<double> face_area;
    getFaceArea(mesh, face_area);

    area_based_edge_operator.resize((int)mesh.n_edges(), TriMesh::Point(0.0, 0.0, 0.0));
    std::vector<double> temp_coef(4, 0.0);
    coef.resize(mesh.n_edges(), temp_coef);
    std::vector<TriMesh::VertexHandle> vertex_handle(4);
    edge_vertex_handle.resize(mesh.n_edges(), vertex_handle);
    for(TriMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); e_it++)
    {
        if(!mesh.is_boundary(*e_it))
        {
            int index = e_it->idx();
            double edge_length = mesh.calc_edge_length(*e_it);

            // get four vertices correspond to edge *e_it
            TriMesh::HalfedgeHandle he = mesh.halfedge_handle(*e_it, 0);
            TriMesh::VertexHandle v1 = mesh.from_vertex_handle(he);
            TriMesh::VertexHandle v3 = mesh.to_vertex_handle(he);
            TriMesh::HalfedgeHandle he_next = mesh.next_halfedge_handle(he);
            TriMesh::VertexHandle v4 = mesh.to_vertex_handle(he_next);

            TriMesh::HalfedgeHandle he_oppo = mesh.opposite_halfedge_handle(he);
            TriMesh::HalfedgeHandle he_oppo_next = mesh.next_halfedge_handle(he_oppo);
            TriMesh::VertexHandle v2 = mesh.to_vertex_handle(he_oppo_next);

            // two faces
            TriMesh::FaceHandle f1 = mesh.face_handle(he);
            TriMesh::FaceHandle f2 = mesh.face_handle(he_oppo);

            // the area of two faces correspond to edge *e_it
            double area134 = face_area[f1.idx()];
            double area123 = face_area[f2.idx()];
            double totalArea = area123 + area134;

            TriMesh::Point p1 = mesh.point(v1);
            TriMesh::Point p2 = mesh.point(v2);
            TriMesh::Point p3 = mesh.point(v3);
            TriMesh::Point p4 = mesh.point(v4);

            TriMesh::Point p12 = p1 - p2;
            TriMesh::Point p13 = p1 - p3;
            TriMesh::Point p14 = p1 - p4;
            TriMesh::Point p23 = p2 - p3;
            TriMesh::Point p34 = p3 - p4;

            // calc coefficient
            temp_coef[0] = (area123 * (p34 | p13) - area134 * (p13 | p23)) / (edge_length * edge_length * totalArea);
            temp_coef[1] = area134 / totalArea;
            temp_coef[2] = (-area123 * (p13 | p14) - area134 * (p12 | p13)) / (edge_length * edge_length * totalArea);
            temp_coef[3] = area123 / totalArea;
            coef[index] = temp_coef;

            vertex_handle[0] = v1;
            vertex_handle[1] = v2;
            vertex_handle[2] = v3;
            vertex_handle[3] = v4;
            edge_vertex_handle[index] = vertex_handle;

            // calc area-based edge operator
            TriMesh::Point pt = p1 * temp_coef[0] + p2 * temp_coef[1] + p3 * temp_coef[2] + p4 * temp_coef[3];
            area_based_edge_operator[index] = pt;
        }
    }
}

void MeshDenoisingViaL0Minimization::solveDelta(std::vector<TriMesh::Point> &area_based_edge_operator, double lambda, double beta,
                                                std::vector<TriMesh::Point> &delta)
{
    delta.resize((int)area_based_edge_operator.size(), TriMesh::Point(0.0, 0.0, 0.0));

    for(int i = 0; i < (int)area_based_edge_operator.size(); i++)
    {
        TriMesh::Point pt = area_based_edge_operator[i];
        if(pt.length() * pt.length() >= lambda/beta)
            delta[i] = pt;
    }
}

void MeshDenoisingViaL0Minimization::getInitialVerticesMatrix(TriMesh &noisy_mesh, Eigen::MatrixXd &initial_vertices_matrix)
{
    initial_vertices_matrix.resize((int)noisy_mesh.n_vertices(), 3);

    for(TriMesh::VertexIter v_it = noisy_mesh.vertices_begin(); v_it != noisy_mesh.vertices_end(); v_it++)
    {
        int index = v_it->idx();
        TriMesh::Point p = noisy_mesh.point(*v_it);
        initial_vertices_matrix(index, 0) = p[0];
        initial_vertices_matrix(index, 1) = p[1];
        initial_vertices_matrix(index, 2) = p[2];
    }
}

void MeshDenoisingViaL0Minimization::solveVertices(TriMesh &mesh, Eigen::MatrixXd &initial_vertices_matrix,
                                                   std::vector< std::vector<TriMesh::VertexHandle> > &edge_vertex_handle,
                                                   std::vector< std::vector<double> > &coef, std::vector<TriMesh::Point> &delta,
                                                   double alpha, double beta)
{
    Eigen::MatrixXd right_term = initial_vertices_matrix;
    Eigen::SparseMatrix<double> coef_matrix((int)mesh.n_vertices(), (int)mesh.n_vertices());

    std::vector< Eigen::Triplet<double> > triple; triple.clear();

    std::map<TriMesh::VertexHandle, double> vertex_coef;
    std::set<TriMesh::EdgeHandle> edge_handle;;
    for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
    {
        edge_handle.clear();
        vertex_coef.clear();
        vertex_coef[*v_it] = 1.0;
        for(TriMesh::VertexFaceIter vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); vf_it++)
        {
            for(TriMesh::FaceEdgeIter fe_it = mesh.fe_iter(*vf_it); fe_it.is_valid(); fe_it++)
            {
                edge_handle.insert(*fe_it);
            }
        }

        TriMesh::Point right(0.0, 0.0, 0.0);
        for(std::set<TriMesh::EdgeHandle>::iterator s_it = edge_handle.begin(); s_it != edge_handle.end(); s_it++)
        {
            if(!mesh.is_boundary(*s_it))
            {
                int index = (*s_it).idx();
                TriMesh::VertexHandle v1 = edge_vertex_handle[index][0],
                        v2 = edge_vertex_handle[index][1],
                        v3 = edge_vertex_handle[index][2],
                        v4 = edge_vertex_handle[index][3];
                double coe1 = coef[index][0],
                        coe2 = coef[index][1],
                        coe3 = coef[index][2],
                        coe4 = coef[index][3];
                TriMesh::Point temp_delta = delta[index];
                if(v1 == *v_it)
                {
                    vertex_coef[v1] = vertex_coef[v1] + alpha + beta * coe1 * coe1;
                    vertex_coef[v2] = vertex_coef[v2] - alpha + beta * coe1 * coe2;
                    vertex_coef[v3] = vertex_coef[v3] + alpha + beta * coe1 * coe3;
                    vertex_coef[v4] = vertex_coef[v4] - alpha + beta * coe1 * coe4;
                    right += temp_delta * beta * coe1;
                }
                else if(v2 == *v_it)
                {
                    vertex_coef[v1] = vertex_coef[v1] - alpha + beta * coe2 * coe1;
                    vertex_coef[v2] = vertex_coef[v2] + alpha + beta * coe2 * coe2;
                    vertex_coef[v3] = vertex_coef[v3] - alpha + beta * coe2 * coe3;
                    vertex_coef[v4] = vertex_coef[v4] + alpha + beta * coe2 * coe4;
                    right += temp_delta * beta * coe2;
                }
                else if(v3 == *v_it)
                {
                    vertex_coef[v1] = vertex_coef[v1] + alpha + beta * coe3 * coe1;
                    vertex_coef[v2] = vertex_coef[v2] - alpha + beta * coe3 * coe2;
                    vertex_coef[v3] = vertex_coef[v3] + alpha + beta * coe3 * coe3;
                    vertex_coef[v4] = vertex_coef[v4] - alpha + beta * coe3 * coe4;
                    right += temp_delta * beta * coe3;
                }
                else if(v4 == *v_it)
                {
                    vertex_coef[v1] = vertex_coef[v1] - alpha + beta * coe4 * coe1;
                    vertex_coef[v2] = vertex_coef[v2] + alpha + beta * coe4 * coe2;
                    vertex_coef[v3] = vertex_coef[v3] - alpha + beta * coe4 * coe3;
                    vertex_coef[v4] = vertex_coef[v4] + alpha + beta * coe4 * coe4;
                    right += temp_delta * beta * coe4;
                }
            }
        }
        right_term(v_it->idx(), 0) += right[0];
        right_term(v_it->idx(), 1) += right[1];
        right_term(v_it->idx(), 2) += right[2];

        for(std::map<TriMesh::VertexHandle, double>::iterator m_it = vertex_coef.begin(); m_it != vertex_coef.end(); m_it++)
        {
            triple.push_back(Eigen::Triplet<double>(v_it->idx(), m_it->first.idx(), m_it->second));
        }
    }
    coef_matrix.setFromTriplets(triple.begin(), triple.end());

    Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
    solver.analyzePattern(coef_matrix);
    solver.factorize(coef_matrix);
    Eigen::MatrixXd vertices_term = solver.solve(right_term);

    for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
    {
        int index = v_it->idx();
        TriMesh::Point pt = TriMesh::Point(vertices_term(index,0), vertices_term(index,1), vertices_term(index,2));
        mesh.set_point(*v_it, pt);
    }
}
