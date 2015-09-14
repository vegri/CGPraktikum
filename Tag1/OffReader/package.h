#ifndef PACKAGE_H
#define PACKAGE_H
#include "vecmath.h"
#include <vector>

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

private:
    void init();
    double height, width, depth;
    Vector3d center;
    Vector4d color;
    Quat4d rot;
    vecvec3d corners;

    uint serial;
    static uint next_serial;
};

#endif // PACKAGE_H
