#ifndef PACKAGE_H
#define PACKAGE_H
#include "vecmath.h"
#include <vector>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <limits.h>
#include "obb.h"

#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

typedef std::vector<Vector3d> vecvec3d;
typedef unsigned int uint;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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
    Vector3d packageInBox(AABB &box);
    void setRot(Quat4d rot_p);
    vecvec3d getCorners();
    Vector3d getCollDir();
    Vector3d penetration(const vecvec3d T);
    Quat4d getRot();
    void rotate(Quat4d rot_p);
    void markRot();
    void rotateMarked(Quat4d rot_p);
    double getDiameter();
    Vector3d getCenter();
    void setCenter(Vector3d center_p);
    void pick(bool picked);
    Vector3d getRotProjection();
    void removeRot();
    void getDist(Vector3d loc_origin, Vector3d direction, double epsilon, double &vert_dist, double &parallel_dist);
    void getDistEdge(Vector3d loc_origin, Vector3d direction, uint i, uint j, double &vert_dist, double &parallel_dist);
    void setMoveDir(bool move_dir_p);
    bool getHit(Vector3d loc_origin, Vector3d direction, Vector3d &hit, double &parallel_dist);
    void solve3dLinearSystem(const Matrix4d &m, Vector3d &x, const Vector3d &s);
    void getIntersectionLinePlane(const Vector3d &loc,  const Vector3d &dir,
                                           const Vector3d &foot, const Vector3d &plane_vec1, const Vector3d &plane_vec2,
                                           double &mu, double &lam1, double &lam2);
    void getDistCircleLine(Vector3d loc_origin, Vector3d direction, double epsilon, double &vert_dist, double &parallel_dist, Vector3d &hit);
    double getCircleRad();

    bool intersectAxis(Vector3d &v, vecvec3d &a, vecvec3d &b, Vector3d &alpha, Vector3d &beta, Vector3d &dc);
    bool resolveCollision(Package &B);
    bool isPicked();
    Vector3d normal;
    Vector3d base;
    vecvec3d axis;
    Vector3d halflength;
    bool collision;
    void setCollision(Package &other);
    void resetCollision();
    double zoom_val;

    bool intersect(Package &B);
    bool intersect(AABB &B);
    friend bool AABB::intersect(Package &A,AABB &B);
private:
    void init();
    double height, width, depth;
    Vector3d center;
    Vector3d move_dir;
    int move_in_dir;
    int rot_dir;
    bool move_dir_b;
    bool rot_dir_b;
    bool rot_ball_b;
    bool picked;
    double circle_rad;
    Vector4d color;
    Quat4d rot;
    //Necessary because trackball rotates relative to picked point
    Quat4d rot_old;
    vecvec3d corners;

    uint serial;
    static uint next_serial;
    static Vector4d pickColor;
    static Vector4d pickColorColl;

    Vector3d collDir;

    //DEBUG
    bool write_lines;

public:
    vecvec3d d_ray_lines,d_ray_points;
};

#endif // PACKAGE_H
