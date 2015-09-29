#ifndef MESHDENOISINGVIAL0MINIMIZATION_H
#define MESHDENOISINGVIAL0MINIMIZATION_H

/*
 @brief: "Mesh Denoising via L0 Minimization" class
 @reference: Mesh Denoising via L0 Minimization, TOG 2013
*/

#include "MeshDenoisingBase.h"
#include <vector>
#include <map>
#include <set>
#include "Eigen/Dense"
#include "Eigen/Sparse"

class MeshDenoisingViaL0Minimization : public MeshDenoisingBase
{
public:
    MeshDenoisingViaL0Minimization(DataManager *_data_manager, ParameterSet *_parameter_set);
    ~MeshDenoisingViaL0Minimization() {}

private:
    void denoise();
    void initParameters();

    double getAverageDihedralAngle(TriMesh &mesh);
    void calculateAreaBasedEdgeOperator(TriMesh &mesh,
                                        std::vector<TriMesh::Point> &area_based_edge_operator,
                                        std::vector<std::vector<TriMesh::VertexHandle> > &edge_vertex_handle,
                                        std::vector< std::vector<double> > &coef);
    void solveDelta(std::vector<TriMesh::Point> &area_based_edge_operator, double lambda, double beta,
                    std::vector<TriMesh::Point> &delta);
    void getInitialVerticesMatrix(TriMesh &noisy_mesh, Eigen::MatrixXd &initial_vertices_matrix);
    void solveVertices(TriMesh &mesh, Eigen::MatrixXd &initial_vertices_matrix,
                       std::vector< std::vector<TriMesh::VertexHandle> > &edge_vertex_handle,
                       std::vector< std::vector<double> > &coef, std::vector<TriMesh::Point> &delta,
                       double alpha, double beta);
};

#endif // MESHDENOISINGVIAL0MINIMIZATION_H
