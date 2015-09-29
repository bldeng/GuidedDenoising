#include "meshexaminer.h"
#include "glwrapper.h"
#include <QFileDialog>
#include <QString>

MeshExaminer::MeshExaminer()
    :draw_points_status_(false), draw_edges_status_(false), draw_faces_status_(true)
{

}

MeshExaminer::~MeshExaminer()
{

}

void MeshExaminer::resetMesh(const TriMesh &_mesh, bool _need_normalize)
{    
    updateMesh(_mesh);

    if(_need_normalize){
        TriMesh::Point max_coord, min_coord;
        if(meshBoundingBox(min_coord, max_coord)){
            setScene( (OpenMesh::Vec3f)((min_coord + max_coord)*0.5), 0.5*(max_coord - min_coord).norm());
        }
        else{
            setScene( OpenMesh::Vec3f(0, 0, 0), 1);
        }
    }
}

void MeshExaminer::updateMesh(const TriMesh &_mesh)
{
    mesh_show_ = _mesh;
    mesh_show_.request_face_normals();
    mesh_show_.request_vertex_normals();
    mesh_show_.update_normals();
}

void MeshExaminer::draw()
{
    if (draw_points_status_)
    {
        glBegin(GL_POINTS);
        for (TriMesh::VertexIter v_it = mesh_show_.vertices_begin();
             v_it != mesh_show_.vertices_end(); v_it++)
        {
            TriMesh::Normal normal = mesh_show_.normal(*v_it);
            TriMesh::Point point = mesh_show_.point(*v_it);
            glColor3f(0.0f,0.0f,1.0f);
            glNormal3f(normal[0], normal[1], normal[2]);
            glVertex3f(point[0], point[1], point[2]);
        }
        glEnd();
    }

    if (draw_faces_status_)
    {
        glBegin(GL_TRIANGLES);
        for (TriMesh::FaceIter f_it = mesh_show_.faces_begin();
             f_it != mesh_show_.faces_end(); f_it++)
        {
            TriMesh::Normal normal = mesh_show_.normal(*f_it);

            for(TriMesh::FaceHalfedgeIter fh_it = mesh_show_.fh_iter(*f_it);
                fh_it.is_valid(); fh_it++)
            {
                TriMesh::VertexHandle toV = mesh_show_.to_vertex_handle(*fh_it);
                TriMesh::Point point = mesh_show_.point(toV);
                glColor3f(0.7f,0.7f,0.0f);
                glNormal3f(normal[0], normal[1], normal[2]);
                glVertex3f(point[0], point[1], point[2]);
            }
        }
        glEnd();
    }

    if (draw_edges_status_)
    {
        for(TriMesh::FaceIter f_it = mesh_show_.faces_begin();
            f_it != mesh_show_.faces_end(); f_it++)
        {
            glBegin(GL_LINE_LOOP);
            for(TriMesh::FaceHalfedgeIter fh_it = mesh_show_.fh_iter(*f_it);
                fh_it.is_valid(); fh_it++)
            {
                TriMesh::VertexHandle toV = mesh_show_.to_vertex_handle(*fh_it);
                TriMesh::Normal normal = mesh_show_.normal(toV);
                TriMesh::Point point = mesh_show_.point(toV);
                glColor3f(0.3f, 0.3f, 0.3f);
                glNormal3f(normal[0], normal[1], normal[2]);
                glVertex3f(point[0], point[1], point[2]);
            }
            glEnd();
        }
    }

}

bool MeshExaminer::meshBoundingBox(TriMesh::Point &min_coord, TriMesh::Point &max_coord)
{
    if(mesh_show_.n_vertices() == 0){
        return false;
    }

    TriMesh::ConstVertexIter cv_it = mesh_show_.vertices_begin(), cv_end(mesh_show_.vertices_end());
    min_coord = mesh_show_.point(*cv_it);
    max_coord = min_coord;

    for( ++ cv_it ; cv_it != cv_end; ++ cv_it){
        min_coord.minimize(mesh_show_.point(*cv_it));
        max_coord.maximize(mesh_show_.point(*cv_it));
    }

    return true;
}
