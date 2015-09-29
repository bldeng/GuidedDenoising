#ifndef GLEXAMINER_H
#define GLEXAMINER_H

#include <OpenMesh/Core/Geometry/VectorT.hh>
class QMouseEvent;
class QWheelEvent;

class GLExaminer
{
public:
    GLExaminer();
    virtual ~GLExaminer();

    void  setScene(const OpenMesh::Vec3f &_center, float _radius);
    void  viewAll();

    virtual void draw() = 0;
    virtual void init();
    virtual void mousePressEvent(QMouseEvent*);
    virtual void mouseMoveEvent(QMouseEvent*);
    virtual void mouseReleaseEvent(QMouseEvent*);
    virtual void wheelEvent(QWheelEvent*);
    virtual void reshape(int _w, int _h);

protected:

    // updates projection matrix
    void updateProjectionMatrix();
    // translate the scene and update modelview matrix
    void translate(const OpenMesh::Vec3f& _trans);
    // rotate the scene (around its center) and update modelview matrix
    void rotate(const OpenMesh::Vec3f& _axis, float _angle);

    // virtual trackball: map 2D screen point to unit sphere
    bool mapToSphere(const OpenMesh::Vec2i &_point, OpenMesh::Vec3f &_result);

    // mouse processing functions
    void rotation(int _x, int _y);
    void translation(int _x, int _y);
    void zoom(float _x);

protected:
    int  width_, height_;

    // scene position and dimension
    OpenMesh::Vec3f    center_;
    float    radius_;

    // projection parameters
    float    near_, far_, fovy_;

    // OpenGL matrices
    double   projection_matrix_[16],
    modelview_matrix_[16];

    // trackball helpers
    OpenMesh::Vec2i    last_point_2D_;
    OpenMesh::Vec3f    last_point_3D_;
    bool     last_point_ok_;
    bool     left_button_down_, right_button_down_;
    int      modifiers_;
};

#endif // GLEXAMINER_H
