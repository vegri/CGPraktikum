#ifndef PACKAGE_H
#define PACKAGE_H
#include "vecmath.h"
#include <vector>
#include <iostream>

#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

typedef std::vector<Vector3d> vecvec3d;
typedef unsigned int uint;

class Package
{
public:
    Package();
    Package(double height_p,double width_p,double depth_p);
    Package(double &height_p,double &width_p,double &depth_p, Vector3d &center_p);
    Package(double &height_p,double &width_p,double &depth_p, Vector3d &center_p, Quat4d &rot_p);
    void draw();
    void resetColor();
    void setColor(Vector4d color_p);
    void move(Vector3d move_p);
    double getDiameter();
    Vector3d getCenter();
    void getDist(Vector3d loc_origin, Vector3d direction, double epsilon, double &vert_dist, double &parallel_dist);
    void getDistEdge(Vector3d loc_origin, Vector3d direction, uint i, uint j, double &vert_dist, double &parallel_dist);
    void setMoveDir(bool move_dir_p);
    bool getHit(Vector3d loc_origin, Vector3d direction, double epsilon, Vector3d &hit, double &parallel_dist);
    void solve3dLinearSystem(const Matrix4d &m, Vector3d &x, const Vector3d &s);
    void getIntersectionLinePlane(const Vector3d &loc,  const Vector3d &dir,
                                           const Vector3d &foot, const Vector3d &plane_vec1, const Vector3d &plane_vec2,
                                           double &mu, double &lam1, double &lam2);
private:
    void init();
    double height, width, depth;
    Vector3d center;
    Vector3d move_dir;
    int move_in_dir;
    bool move_dir_b;
    Vector4d color;
    Quat4d rot;
    vecvec3d corners;

    uint serial;
    static uint next_serial;

    //DEBUG
    bool write_lines;
    vecvec3d d_ray_lines,d_ray_points;
};

#endif // PACKAGE_H
