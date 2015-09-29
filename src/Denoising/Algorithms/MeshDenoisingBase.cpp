#include "MeshDenoisingBase.h"

MeshDenoisingBase::MeshDenoisingBase(DataManager *_data_manager, ParameterSet *_parameter_set)
{
    if((_data_manager == NULL) || (_parameter_set == NULL))
        return;

    data_manager_ = _data_manager;
    parameter_set_ = _parameter_set;
}

double MeshDenoisingBase::getAverageEdgeLength(TriMesh &mesh)
{
    double average_edge_length = 0.0;
    for(TriMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); e_it++)
        average_edge_length += mesh.calc_edge_length(*e_it);
    double edgeNum = (double)mesh.n_edges();
    average_edge_length /= edgeNum;

    return average_edge_length;
}

void MeshDenoisingBase::getFaceArea(TriMesh &mesh, std::vector<double> &area)
{
    area.resize(mesh.n_faces());

    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        std::vector<TriMesh::Point> point;
        point.resize(3); int index = 0;
        for(TriMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); fv_it++)
        {
            point[index] = mesh.point(*fv_it);
            index++;
        }
        TriMesh::Point edge1 = point[1] - point[0];
        TriMesh::Point edge2 = point[1] - point[2];
        double S = 0.5 * (edge1 % edge2).length();
        area[(*f_it).idx()] = S;
    }
}

void MeshDenoisingBase::getFaceCentroid(TriMesh &mesh, std::vector<TriMesh::Point> &centroid)
{
    centroid.resize(mesh.n_faces(), TriMesh::Point(0.0, 0.0, 0.0));
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        TriMesh::Point pt = mesh.calc_face_centroid(*f_it);
        centroid[(*f_it).idx()] = pt;
    }
}

void MeshDenoisingBase::getFaceNormal(TriMesh &mesh, std::vector<TriMesh::Normal> &normals)
{
    mesh.request_face_normals();
    mesh.update_face_normals();

    normals.resize(mesh.n_faces());
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        TriMesh::Normal n = mesh.normal(*f_it);
        normals[f_it->idx()] = n;
    }
}

void MeshDenoisingBase::getFaceNeighbor(TriMesh &mesh, TriMesh::FaceHandle fh, FaceNeighborType face_neighbor_type, std::vector<TriMesh::FaceHandle> &face_neighbor)
{
    face_neighbor.clear();
    if(face_neighbor_type == kEdgeBased)
    {
        for(TriMesh::FaceFaceIter ff_it = mesh.ff_iter(fh); ff_it.is_valid(); ff_it++)
            face_neighbor.push_back(*ff_it);
    }
    else if(face_neighbor_type == kVertexBased)
    {
        std::set<int> neighbor_face_index; neighbor_face_index.clear();

        for(TriMesh::FaceVertexIter fv_it = mesh.fv_begin(fh); fv_it.is_valid(); fv_it++)
        {
            for(TriMesh::VertexFaceIter vf_it = mesh.vf_iter(*fv_it); vf_it.is_valid(); vf_it++)
            {
                if((*vf_it) != fh)
                    neighbor_face_index.insert(vf_it->idx());
            }
        }

        for(std::set<int>::iterator iter = neighbor_face_index.begin(); iter != neighbor_face_index.end(); ++ iter)
        {
            face_neighbor.push_back(TriMesh::FaceHandle(*iter));
        }
    }
}

void MeshDenoisingBase::getAllFaceNeighbor(TriMesh &mesh, std::vector<std::vector<TriMesh::FaceHandle> > &all_face_neighbor, FaceNeighborType face_neighbor_type, bool include_central_face)
{
    all_face_neighbor.resize(mesh.n_faces());
    for(TriMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++)
    {
        std::vector<TriMesh::FaceHandle> face_neighbor;
        getFaceNeighbor(mesh, *f_it, face_neighbor_type, face_neighbor);
        if(include_central_face) face_neighbor.push_back(*f_it);
        all_face_neighbor[f_it->idx()] = face_neighbor;
    }
}

void MeshDenoisingBase::updateVertexPosition(TriMesh &mesh, std::vector<TriMesh::Normal> &filtered_normals, int iteration_number, bool fixed_boundary)
{
    std::vector<TriMesh::Point> new_points(mesh.n_vertices());

    std::vector<TriMesh::Point> centroid;

    for(int iter = 0; iter < iteration_number; iter++)
    {
        getFaceCentroid(mesh, centroid);
        for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
        {
            TriMesh::Point p = mesh.point(*v_it);
            if(fixed_boundary && mesh.is_boundary(*v_it))
            {
                new_points.at(v_it->idx()) = p;
            }
            else
            {
                double face_num = 0.0;
                TriMesh::Point temp_point(0.0, 0.0, 0.0);
                for(TriMesh::VertexFaceIter vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); vf_it++)
                {
                    TriMesh::Normal temp_normal = filtered_normals[vf_it->idx()];
                    TriMesh::Point temp_centroid = centroid[vf_it->idx()];
                    temp_point += temp_normal * (temp_normal | (temp_centroid - p));
                    face_num++;
                }
                p += temp_point / face_num;

                new_points.at(v_it->idx()) = p;
            }
        }

        for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++)
            mesh.set_point(*v_it, new_points[v_it->idx()]);
    }
}

double MeshDenoisingBase::getMeanSquareAngleError(TriMesh &DenoisedMesh, TriMesh &OriginalMesh)
{
    DenoisedMesh.request_face_normals();
    DenoisedMesh.update_face_normals();

    OriginalMesh.request_face_normals();
    OriginalMesh.update_face_normals();

    double mean_square_angle_error = 0.0;
    for (TriMesh::FaceIter f_it1 = DenoisedMesh.faces_begin(), f_it2 = OriginalMesh.faces_begin();
        f_it1 != DenoisedMesh.faces_end(); f_it1++, f_it2++ )
    {
        TriMesh::Normal normal1 = DenoisedMesh.normal(*f_it1);
        TriMesh::Normal normal2 = OriginalMesh.normal(*f_it2);
        double cross_value = normal1 | normal2;
        cross_value = std::min(1.0, std::max(cross_value, -1.0));
        mean_square_angle_error += std::acos(cross_value) * 180.0 / M_PI;
    }

    return mean_square_angle_error / (double)DenoisedMesh.n_faces();
}
