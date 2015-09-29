#include "NonIterativeFeaturePreservingMeshFiltering.h"

NonIterativeFeaturePreservingMeshFiltering::NonIterativeFeaturePreservingMeshFiltering(DataManager *_data_manager, ParameterSet *_parameter_set)
    : MeshDenoisingBase(_data_manager, _parameter_set)
{
    initParameters();
}

void NonIterativeFeaturePreservingMeshFiltering::denoise()
{
    // get data
    TriMesh mesh = data_manager_->getNoisyMesh();

    if(mesh.n_vertices() == 0)
        return;

    // get parameter
    double sigma_g_mean_edge_length_ratio, sigma_f_mean_edge_length_ratio;
    if(!parameter_set_->getValue(QString("sigma_f/mean_edge_length"), sigma_f_mean_edge_length_ratio))
        return;
    if(!parameter_set_->getValue(QString("sigma_g/mean_edge_length"), sigma_g_mean_edge_length_ratio))
        return;

    // mesh denoise
    double mean_edge_length = getAverageEdgeLength(mesh);
    std::vector<double> face_area;
    getFaceArea(mesh, face_area);
    std::vector<TriMesh::Point> face_centroid;
    getFaceCentroid(mesh, face_centroid);

    double sigma_f = sigma_f_mean_edge_length_ratio * mean_edge_length;
    double sigma_g = sigma_g_mean_edge_length_ratio * mean_edge_length;

    std::vector<TriMesh::Normal> mollified_normals;
    mollifiedNormals(mesh, face_centroid, face_area, sigma_f, mollified_normals);

    std::vector<TriMesh::Point> new_point(mesh.n_vertices());
    for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
        new_point[v_it->idx()] = mesh.point(*v_it);

    for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
    {
        TriMesh::Point pt = mesh.point(*v_it);

        std::vector<TriMesh::FaceHandle> vertex_face_neighbor;
        getVertexFaceNeighbor(mesh, face_centroid, *v_it, sigma_f, vertex_face_neighbor);

        TriMesh::Point temp_point(0.0, 0.0, 0.0);
        double weight_sum = 0.0;

        for(int i = 0; i <(int)vertex_face_neighbor.size(); i++)
        {
            TriMesh::Point centroid = face_centroid[vertex_face_neighbor[i].idx()];
            TriMesh::Normal normal = mollified_normals[vertex_face_neighbor[i].idx()];
            TriMesh::Point projection_point = projectPoint(pt, centroid, normal);

            double distance_spatial = (centroid - pt).length();
            double weight_spatial = std::exp(- 0.5 * distance_spatial * distance_spatial /(sigma_f * sigma_f));

            double distance_influence = (projection_point - pt).length();
            double weight_influence = std::exp(- 0.5 * distance_influence * distance_influence /(sigma_g * sigma_g));

            double area = face_area[vertex_face_neighbor[i].idx()];

            temp_point += projection_point * area * weight_spatial * weight_influence;
            weight_sum += area * weight_spatial * weight_influence;
        }
        temp_point /= weight_sum;

        new_point.at((*v_it).idx()) = temp_point;
    }

    for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
        mesh.set_point(*v_it, new_point[(*v_it).idx()]);

    // update data
    data_manager_->setMesh(mesh);
    data_manager_->setDenoisedMesh(mesh);
}

void NonIterativeFeaturePreservingMeshFiltering::initParameters()
{
    parameter_set_->removeAllParameter();
    parameter_set_->addParameter(QString("sigma_f/mean_edge_length"), 1.0, QString("sigma_f/mean_edge_length"), QString("Standard deviation of spatial weight."),
                                 true, 0.001, 10.000);
    parameter_set_->addParameter(QString("sigma_g/mean_edge_length"), 1.0, QString("sigma_g/mean_edge_length"), QString("Standard deviation of nfluence weight."),
                                 true, 0.001, 10.000);

    parameter_set_->setName("Non-Iterative, Feature Preserving Mesh Filtering");
    parameter_set_->setLabel("Non-Iterative, Feature Preserving Mesh Filtering");
    parameter_set_->setIntroduction("Non-Iterative, Feature Preserving Mesh Filtering -- Parameters");
}

void NonIterativeFeaturePreservingMeshFiltering::getVertexFaceNeighbor(TriMesh &mesh, std::vector<TriMesh::Point> &face_centroid, TriMesh::VertexHandle vh, double r,
                                                                       std::vector<TriMesh::FaceHandle> &vertex_face_neighbor)
{
    double radius = 2.0 * r;
    TriMesh::Point point = mesh.point(vh);

    std::vector<bool> mark(mesh.n_faces(), false);
    vertex_face_neighbor.clear();
    std::queue<TriMesh::FaceHandle> queue_face_handle;
    for(TriMesh::VertexFaceIter vf_it = mesh.vf_iter(vh); vf_it.is_valid(); vf_it++)
    {
        TriMesh::Point centroid = face_centroid[(*vf_it).idx()];
        double length = (point - centroid).length();
        if( length <= radius )
            queue_face_handle.push(*vf_it);
        mark[(*vf_it).idx()] = true;
    }

    std::vector<TriMesh::FaceHandle> face_neighbor;

    while(!queue_face_handle.empty())
    {
        TriMesh::FaceHandle fh = queue_face_handle.front();
        vertex_face_neighbor.push_back(fh);
        queue_face_handle.pop();
        getFaceNeighbor(mesh, fh, kVertexBased, face_neighbor);
        for(int i = 0; i < (int)face_neighbor.size(); i++)
        {
            TriMesh::FaceHandle temp_fh = face_neighbor[i];
            if(mark[temp_fh.idx()] == false)
            {
                TriMesh::Point centroid = face_centroid[temp_fh.idx()];
                double length = (point - centroid).length();
                if(length <= radius)
                    queue_face_handle.push(temp_fh);
                mark[temp_fh.idx()] = true;
            }
        }
    }
}

void NonIterativeFeaturePreservingMeshFiltering::mollifiedNormals(TriMesh &mesh, std::vector<TriMesh::Point> &face_centroid, std::vector<double> &face_area, double sigma_f,
                                                                  std::vector<TriMesh::Normal> &mollified_normals)
{
    TriMesh temp_mesh = mesh;

    double sigma_m = sigma_f / 2.0;

    for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
    {
        TriMesh::Point pt = mesh.point(*v_it);

        std::vector<TriMesh::FaceHandle> vertex_face_neighbor;
        getVertexFaceNeighbor(mesh, face_centroid, *v_it, sigma_f, vertex_face_neighbor);

        TriMesh::Point new_point(0.0, 0.0, 0.0);
        double weight_sum = 0.0;
        for(int i = 0; i < (int)vertex_face_neighbor.size(); i++)
        {
            TriMesh::Point centroid = face_centroid[vertex_face_neighbor[i].idx()];

            double dis = (centroid - pt).length();
            double weight = std::exp(- 0.5 * dis * dis / (sigma_m * sigma_m));

            double area = face_area[vertex_face_neighbor[i].idx()];
            new_point += centroid * area * weight;
            weight_sum += area * weight;
        }
        new_point /= weight_sum;

        int vertex_index = (*v_it).idx();
        TriMesh::VertexHandle vh = temp_mesh.vertex_handle(vertex_index);
        temp_mesh.set_point(vh, new_point);
    }

    mollified_normals.resize(mesh.n_faces());
    temp_mesh.request_face_normals();
    temp_mesh.update_face_normals();
    for(TriMesh::FaceIter f_it = temp_mesh.faces_begin(); f_it != temp_mesh.faces_end(); f_it++)
        mollified_normals[(*f_it).idx()] = temp_mesh.normal(*f_it);
}

TriMesh::Point NonIterativeFeaturePreservingMeshFiltering::projectPoint(TriMesh::Point pt, TriMesh::Point centroid, TriMesh::Normal n)
{
    return pt - n * ( (pt - centroid) | n );
}
