#ifndef BOUNDINGBODY_H
#define BOUNDINGBODY_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>

#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

#include "vecmath.h"

typedef unsigned int uint;
typedef std::vector<Vector3d> vecvec3d;
typedef std::vector<uint> vecuint;
typedef std::vector<vecuint> vecvecuint;

class BoundingBody
{
public:
    BoundingBody();
    void draw_mesh();
    void draw_mesh_collision();
    void draw_mesh_no_collision();
    void setBoxColor(Vector3d color_p);
    void setCollisionColor(Vector3d color_p);
    Vector3d getCenter();
    Vector3d getBodyCenter();
    Vector3d getColor();
    vecvec3d *getP();
    vecvecuint getInd();
    void setRotation(Quat4d q);
    void setZoom(double zoom_p);
    void setCenter(Vector3d center_p);
    void addCollision(BoundingBody &b);
    void clearCollision();
    bool intersectTrianlge(vecvec3d &a_p, vecvec3d &b_p);

    bool planeTest(const Vector3d &v, const vecvec3d &a, const vecvec3d &b, const Vector3d &dc);
    void move(Vector3d motion);
    void zoom(double z_p);
    void rotate(Quat4d rotation);
    virtual void draw()=0;
    Quat4d q_now;
    double zoom_val;

    //virtual bool intersect(BoundingBody &B)=0;
    //virtual bool intersect(BoundingBody &B);

protected:
    Vector3d min, max, center, bodycenter;
    vecvec3d quad;
    std::vector<BoundingBody*> collision;
    bool selected;

    vecvec3d *p;
    vecvecuint ind;

    Vector3d box_color,collision_color,selected_color,selected_collision_color;
};

#endif // BOUNDINGBODY_H
