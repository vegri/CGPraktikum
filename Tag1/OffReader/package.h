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
private:
    void init();
    double height, width, depth;
    Vector3d center;
    Quat4d rot;
    vecvec3d corners;
};

#endif // PACKAGE_H
