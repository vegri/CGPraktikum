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
typedef std::vector<Vector3d> vec3dd;
typedef std::vector<int> vecintd;

class BoundingBody
{
public:
    BoundingBody();
    void draw_mesh();
    void draw_mesh_collision();
    void draw_mesh_no_collision();
    void setColor(Vector3d color_p);
    void setBoxColor(Vector3d color_p);
    void setCollisionColor(Vector3d color_p);
    Vector3d getCenter();
    Vector3d getBodyCenter();
    Vector3d getColor();
    vec3dd getP();
    vecintd getInd();
    void setRotation(Quat4d q);
    void setZoom(double zoom_p);
    void setCenter(Vector3d center_p);
    void addCollision(BoundingBody &b);
    void clearCollision();
    bool intersectTrianlge(vec3dd &a_p, vec3dd &b_p);

    bool planeTest(const Vector3d &v, const vec3dd &a, const vec3dd &b, const Vector3d &dc);
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
    vec3dd quad;    
    std::vector<BoundingBody*> collision;

    vec3dd p;
    vecintd ind;
    Vector3d color,box_color,collision_color;
    //Quat4d q_now;
};

#endif // BOUNDINGBODY_H
