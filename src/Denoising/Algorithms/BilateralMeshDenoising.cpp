#include "BilateralMeshDenoising.h"

BilateralMeshDenoising::BilateralMeshDenoising(DataManager *_data_manager, ParameterSet *_parameter_set)
    : MeshDenoisingBase(_data_manager, _parameter_set)
{
    initParameters();
}

void BilateralMeshDenoising::denoise()
{
    // get data
    TriMesh mesh = data_manager_->getNoisyMesh();

    if(mesh.n_vertices() == 0)
        return;

    // get paramters
    int iteration_number;
    if(!parameter_set_->getValue(QString("Iteration Num."), iteration_number))
        return;

    // mesh denoise
    std::vector<TriMesh::Point> points;
    points.resize(mesh.n_vertices());

    for(int iter = 0; iter < iteration_number; iter++)
    {
        mesh.request_face_normals();
        mesh.request_vertex_normals();
        mesh.update_normals();

        double sigma_c, sigma_s;

        int index = 0;
        for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
        {
            // calculate sigma_c
            TriMesh::Point pi = mesh.point(*v_it);
            TriMesh::Normal ni = mesh.normal(*v_it);
            double max = 1e10;
            for(TriMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); vv_it++)
            {
                TriMesh::Point pj = mesh.point(*vv_it);
                double length = (pi - pj).length();
                if(length < max) max = length;
            }
            sigma_c = max;

            std::vector<TriMesh::VertexHandle> vertex_neighbor;
            getAdaptiveVertexNeighbor(mesh, *v_it, sigma_c, vertex_neighbor);

            // calculate sigma_s
            double average_off_set = 0;
            std::vector<double> off_set_dis;
            off_set_dis.clear();
            for(int i = 0; i < (int)vertex_neighbor.size(); i++)
            {
                TriMesh::Point pj = mesh.point(vertex_neighbor[i]);

                double t = (pj - pi) | ni;
                t = sqrt(t*t);
                average_off_set += t;
                off_set_dis.push_back(t);
            }
            average_off_set = average_off_set / (double)vertex_neighbor.size();
            double offset = 0;
            for(int j = 0; j < (int)off_set_dis.size(); j++)
                offset += (off_set_dis[j] - average_off_set) * (off_set_dis[j] - average_off_set);
            offset /= (double)off_set_dis.size();

            sigma_s = (sqrt(offset) < 1.0e-12) ? (sqrt(offset) + 1.0e-12) : sqrt(offset);

            double sum = 0; double normalizer = 0;
            for(int iv = 0; iv < (int)vertex_neighbor.size(); iv++)
            {
                TriMesh::Point pj = mesh.point(vertex_neighbor[iv]);

                double t = (pi - pj).length();
                double h = (pj - pi) | ni;
                double wc = std::exp(-0.5*t*t/(sigma_c *sigma_c));
                double ws = std::exp(-0.5*h*h/(sigma_s *sigma_s));
                sum += wc * ws * h;
                normalizer += wc * ws;
            }
            points[index] = pi + ni * (sum / normalizer);
            index++;
        }
        index = 0;
        for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
        {
            mesh.set_point(*v_it, points[index]);
            index++;
        }
    }

    // update data
    data_manager_->setMesh(mesh);
    data_manager_->setDenoisedMesh(mesh);
}

void BilateralMeshDenoising::initParameters()
{
    parameter_set_->removeAllParameter();
    parameter_set_->addParameter(QString("Iteration Num."), 5, QString("Iteration Num."), QString("The total iteration number of the algorithm."),
                                 true, 1, 100);
    parameter_set_->setName(QString("Bilateral Mesh Denoising"));
    parameter_set_->setLabel(QString("Bilateral Mesh Denoising"));
    parameter_set_->setIntroduction(QString("Bilateral Mesh Denoising -- Paramters"));
}

void BilateralMeshDenoising::getAdaptiveVertexNeighbor(TriMesh &mesh, TriMesh::VertexHandle vh, double sigma_c,
                                                       std::vector<TriMesh::VertexHandle> &vertex_neighbor)
{
    std::vector<bool> mark(mesh.n_vertices(),false);
    vertex_neighbor.clear();
    std::queue<TriMesh::VertexHandle> queue_vertex_handle;
    mark[vh.idx()] = true;
    queue_vertex_handle.push(vh);
    double radius = 2.0 * sigma_c;
    TriMesh::Point ci = mesh.point(vh);

    while(!queue_vertex_handle.empty())
    {
        TriMesh::VertexHandle vh = queue_vertex_handle.front();
        vertex_neighbor.push_back(vh);
        queue_vertex_handle.pop();
        for(TriMesh::VertexVertexIter vv_it = mesh.vv_iter(vh); vv_it.is_valid(); ++vv_it)
        {
            TriMesh::VertexHandle vh_neighbor = *vv_it;
            if(mark[vh_neighbor.idx()] == false)
            {
                TriMesh::Point cj = mesh.point(vh_neighbor);
                double length = (cj - ci).length();
                if(length <= radius)
                    queue_vertex_handle.push(vh_neighbor);
                mark[vh_neighbor.idx()] = true;
            }
        }
    }
}
