#ifndef PERSPECTIVE_H
#define PERSPECTIVE_H

#include <QMainWindow>
#include <QGLWidget>
#include <QGridLayout>
#include <QFrame>
#include <QActionGroup>
#include <vector>
#include <iostream>
#include <QTimer>
#include <math.h>
#include <queue>
#include "BVT.h"
#include "obb.h"

#include "vecmath.h"
#include "package.h"

typedef std::vector<uint> vecuint;
typedef std::vector<vecuint> vecvecuint;
typedef std::vector<vecvecuint> vecvecvecuint;


#ifndef VECMATH_VERSION
#error "wrong vecmath included, must contain a VECMATH_VERSION macro"
#else
#if VECMATH_VERSION < 2
#error "outdatet vecmath included"
#endif
#endif

#if _MSC_VER
	#include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
	#include <GL/glu.h>
#endif

class CGView;

class CGMainWindow : public QMainWindow {
	Q_OBJECT
public:
	CGMainWindow (QWidget* parent = 0, Qt::WindowFlags flags = Qt::Window);
	~CGMainWindow ();
	QActionGroup *projectionMode;
    void loadPoly(QString filename);

public slots:
  void loadPolyhedron();
  void loadPackage1();
  void loadPackage2();
  void loadPackage3();
  void loadPackage4();
  void loadPackage5();
  void loadPackage6();
  void loadPackage7();
  void loadPackage8();
  void loadPackage9();
  void loadPackage10();
  void loadAllPackages();

private:
	CGView *ogl;	
};

class CGView : public QGLWidget {
	Q_OBJECT
public:
	CGView(CGMainWindow*,QWidget*);
	void initializeGL();
	/** transforms the picture coords (x,y,z) to world coords by 
		inverting the projection and modelview matrix (as it it is 
		after invocation of paintGL) and stores the result in v */
	void worldCoord(int x, int y, int z, Vector3d &v);
    void rot(GLdouble x, GLdouble y, GLdouble z);
    void move(GLdouble x, GLdouble y, GLdouble z);

  Vector3d min, max, center, hit, trackballV, trackballU;
  std::vector<Vector3d> coord; // the coords of the loaded model
  std::vector<Package> packageList;
  std::vector<BVT*> bootList;
  std::vector<std::vector<int> > ifs;   // the faces of the loaded model, ifs[i] contains the indices of the i-th face
	double zoom;
  Quat4d q_old;
  Quat4d q_now;
  uint check(uint act, uint &nx, uint &ny, uint &nz, BVT &tree);
  void createPermissionGrid(BVT &tree,double resolution);

  void keyPressEvent(QKeyEvent *e);
  double resolveCollision(vecvec3d &trans, vecvec3d &rotation);
  double getUtilityValue(vecvec3d &motion, vecvec3d &rotation);
  double updateUtilityValue(uint pack_idx, vecvec3d &motion,vecvec3d &rotation);
  int resolveCollision(Package &B, BVT &Off, bool jumpRot);
  Vector3d collDir;

public slots:
//    void timer();

protected:

	void paintGL();
  void paintModel();
	void resizeGL(int,int);

	void mouseMoveEvent(QMouseEvent*);
	void mousePressEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);
	void wheelEvent(QWheelEvent*);
    void drawIkeaPackage(std::vector<Vector3d> packageList);

  AABB testObb;
  int oldX,oldY;
  int currentWidth, currentHeight;
  void mouseToTrackball(int x, int y, int w, int h);
  void trackball(Vector3d u, Vector3d v, Quat4d &q);
  
  bool animationRunning;
  bool drawObb;
  double timeParameter;
  double typical_diff;

  int animationMode;
  uint picked;
  uint depth;

  bool picked_active;
  bool projRot;
  QFlag mouse_mode;
  const int SLERP;
  const int EULER_ANGLES;
  vecuint grid;
  vecvec3d grid_coord;
  Vector3d grid_diag;
  Vector3d zero;

  //DEBUG
  Vector3d d_ray_f,d_ray_d;
  vecvec3d d_window_points;

};

#endif
