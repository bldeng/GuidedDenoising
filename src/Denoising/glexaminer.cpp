#include "glexaminer.h"
#include "glwrapper.h"
#include <math.h>
#include <QMouseEvent>
#include <QWheelEvent>
#include <iostream>

GLExaminer::GLExaminer()
    :last_point_ok_(false), left_button_down_(false), right_button_down_(false)
{}


GLExaminer::~GLExaminer()
{}

void GLExaminer::init()
{
  // OpenGL state
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//  glClearColor(0.6f, 0.6f, 0.5f, 0.0f);
  glDisable( GL_DITHER );
  glEnable( GL_DEPTH_TEST );
  glEnable(GL_DOUBLEBUFFER);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  // some performance settings
  // glEnable( GL_CULL_FACE );
//  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
//  glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE );


  // material
//  GLfloat mat_a[] = {0.2, 0.2, 0.2, 1.0};
//  GLfloat mat_d[] = {0.4, 0.4, 0.4, 1.0};
//  GLfloat mat_s[] = {0.8, 0.8, 0.8, 1.0};
//  GLfloat shine[] = {128.0};
//  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   mat_a);
//  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat_d);
//  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  mat_s);
//  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shine);


  // lighting
//  glLoadIdentity();

//  GLfloat pos1[] = { 0.1, 0.1, -0.02, 0.0};
//  GLfloat pos2[] = {-0.1, 0.1, -0.02, 0.0};
//  GLfloat pos3[] = { 0.0, 0.0, 0.1, 0.0};
//  GLfloat col1[] = {.05, .05, .6, 1.0};
//  GLfloat col2[] = {.6, .05, .05, 1.0};
//  GLfloat col3[] = {1.0, 1.0, 1.0, 1.0};

//  glEnable(GL_LIGHTING);
//  glEnable(GL_LIGHT0);
//  glLightfv(GL_LIGHT0,GL_POSITION, pos1);
//  glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
//  glLightfv(GL_LIGHT0,GL_SPECULAR, col1);

//  glEnable(GL_LIGHT1);
//  glLightfv(GL_LIGHT1,GL_POSITION, pos2);
//  glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
//  glLightfv(GL_LIGHT1,GL_SPECULAR, col2);

//  glEnable(GL_LIGHT2);
//  glLightfv(GL_LIGHT2,GL_POSITION, pos3);
//  glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
//  glLightfv(GL_LIGHT2,GL_SPECULAR, col3);

  // add
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  // scene pos and size
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix_);
  setScene(OpenMesh::Vec3f(0.0, 0.0, 0.0), 1.0);

  // projection
  near_ = 0.0001f;
  far_  = 1000.0f;
  fovy_ = 45.0f;
}

void GLExaminer::reshape(int _w, int _h)
{
  width_  = _w;
  height_ = _h;
  glViewport(0, 0, _w, _h);
  updateProjectionMatrix();
}

void GLExaminer::updateProjectionMatrix()
{
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluPerspective(fovy_, (GLfloat)width_/(GLfloat)height_, near_, far_);
  glGetDoublev( GL_PROJECTION_MATRIX, projection_matrix_);
  glMatrixMode( GL_MODELVIEW );
}

void GLExaminer::setScene(const OpenMesh::Vec3f& _cog, float _radius )
{
  center_ = _cog;
  radius_ = _radius;

  near_  = 0.01 * radius_;
  far_   = 10.0 * radius_;
  updateProjectionMatrix();

  viewAll();
}
void GLExaminer::viewAll()
{
  translate( OpenMesh::Vec3f( -(modelview_matrix_[0]*center_[0] +
              modelview_matrix_[4]*center_[1] +
              modelview_matrix_[8]*center_[2] +
              modelview_matrix_[12]),
            -(modelview_matrix_[1]*center_[0] +
              modelview_matrix_[5]*center_[1] +
              modelview_matrix_[9]*center_[2] +
              modelview_matrix_[13]),
            -(modelview_matrix_[2]*center_[0] +
              modelview_matrix_[6]*center_[1] +
              modelview_matrix_[10]*center_[2] +
              modelview_matrix_[14] +
              3.0*radius_) ) );
}

bool GLExaminer::mapToSphere(const OpenMesh::Vec2i& _v2D, OpenMesh::Vec3f& _v3D )
{
  if ( (_v2D[0] >= 0) && (_v2D[0] <= width_) &&
       (_v2D[1] >= 0) && (_v2D[1] <= height_) )
  {
    double x  = (double)(_v2D[0] - 0.5*width_)  / (double)width_;
    double y  = (double)(0.5*height_ - _v2D[1]) / (double)height_;
    double sinx         = sin(M_PI * x * 0.5);
    double siny         = sin(M_PI * y * 0.5);
    double sinx2siny2   = sinx * sinx + siny * siny;

    _v3D[0] = sinx;
    _v3D[1] = siny;
    _v3D[2] = sinx2siny2 < 1.0 ? sqrt(1.0 - sinx2siny2) : 0.0;

    return true;
  }
  else
    return false;
}

void GLExaminer::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton || event->button() == Qt::RightButton){
        last_point_2D_ = OpenMesh::Vec2i(event->x(), event->y());
        last_point_ok_ = mapToSphere( last_point_2D_, last_point_3D_ );

        if( event->button() == Qt::LeftButton ){
            left_button_down_ = true;
        }
        if(event->button() == Qt::RightButton){
            right_button_down_ = true;
        }
    }
}

void GLExaminer::mouseReleaseEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton || event->button() == Qt::RightButton){
        last_point_ok_ = false;

        if( event->button() == Qt::LeftButton ){
            left_button_down_ = false;
        }
        if(event->button() == Qt::RightButton){
            right_button_down_ = false;
        }
    }
}

void GLExaminer::wheelEvent(QWheelEvent *event)
{
    float dx = event->delta() * 0.5;
    zoom(dx);
}

void GLExaminer::mouseMoveEvent(QMouseEvent* event)
{
    if(event->buttons() == Qt::LeftButton){
        rotation(event->x(), event->y());
    }
    else if(event->buttons() == Qt::RightButton){
        translation(event->x(), event->y());
    }

    if(event->buttons() == Qt::LeftButton || event->buttons() == Qt::RightButton){
        last_point_2D_ = OpenMesh::Vec2i(event->x(), event->y());
        last_point_ok_ = mapToSphere(last_point_2D_, last_point_3D_);
    }
}

void GLExaminer::rotation(int _x, int _y)
{
  if (last_point_ok_)
  {
    OpenMesh::Vec2i  new_point_2D;
    OpenMesh::Vec3f  new_point_3D;
    bool   new_point_ok;

    new_point_2D = OpenMesh::Vec2i(_x, _y);
    new_point_ok = mapToSphere(new_point_2D, new_point_3D);

    if (new_point_ok)
    {
      OpenMesh::Vec3f axis      = OpenMesh::cross(last_point_3D_, new_point_3D);
      float cos_angle = OpenMesh::dot(last_point_3D_, new_point_3D);

      if (fabs(cos_angle) < 1.0)
      {
        float angle = 2.0*acos(cos_angle) * 180.0 / M_PI;
        rotate(axis, angle);
      }
    }
  }
}

void GLExaminer::translation(int _x, int _y)
{
  float dx = _x - last_point_2D_[0];
  float dy = _y - last_point_2D_[1];

  float z = - ((modelview_matrix_[ 2]*center_[0] +
        modelview_matrix_[ 6]*center_[1] +
        modelview_matrix_[10]*center_[2] +
        modelview_matrix_[14]) /
           (modelview_matrix_[ 3]*center_[0] +
        modelview_matrix_[ 7]*center_[1] +
        modelview_matrix_[11]*center_[2] +
        modelview_matrix_[15]));

  float aspect = (float)width_ / (float)height_;
  float up     = tan(fovy_/2.0f*M_PI/180.f) * near_;
  float right  = aspect*up;

  translate(OpenMesh::Vec3f(2.0*dx/width_*right/near_*z,
          -2.0*dy/height_*up/near_*z,
          0.0f));
}

void GLExaminer::zoom(float _x)
{
  float h  = height_;
  translate(OpenMesh::Vec3f(0.0, 0.0, radius_ * _x * 3.0 / h));
}

void GLExaminer::translate(const OpenMesh::Vec3f& _trans )
{
  glLoadIdentity();
  glTranslated( _trans[0], _trans[1], _trans[2] );
  glMultMatrixd( modelview_matrix_ );
  glGetDoublev( GL_MODELVIEW_MATRIX, modelview_matrix_);
}

void GLExaminer::rotate(const OpenMesh::Vec3f& _axis, float _angle )
{
  OpenMesh::Vec3f t( modelview_matrix_[0]*center_[0] +
       modelview_matrix_[4]*center_[1] +
       modelview_matrix_[8]*center_[2] +
       modelview_matrix_[12],
       modelview_matrix_[1]*center_[0] +
       modelview_matrix_[5]*center_[1] +
       modelview_matrix_[9]*center_[2] +
       modelview_matrix_[13],
       modelview_matrix_[2]*center_[0] +
       modelview_matrix_[6]*center_[1] +
       modelview_matrix_[10]*center_[2] +
       modelview_matrix_[14] );

  glLoadIdentity();
  glTranslatef(t[0], t[1], t[2]);
  glRotated( _angle, _axis[0], _axis[1], _axis[2]);
  glTranslatef(-t[0], -t[1], -t[2]);
  glMultMatrixd(modelview_matrix_);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix_);
}
