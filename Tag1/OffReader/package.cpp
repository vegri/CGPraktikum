#include "package.h"

Package::Package():
    center(Vector3d(0,0,0)),rot(Quat4d(0,0,0,1)),corners(vecvec3d(18))
{
    init();
}

Package::Package(double height_p, double width_p, double depth_p):
    height(height_p),width(width_p),depth(depth_p),center(Vector3d(0,0,0)),rot(Quat4d(0,0,0,1)),corners(vecvec3d(18))
{
    init();
}

Package::Package(double &height_p, double &width_p, double &depth_p, Vector3d &center_p):
        height(height_p),width(width_p),depth(depth_p),center(center_p),rot(Quat4d(0,0,0,1)),corners(vecvec3d(18))
{
    init();
}

Package::Package(double &height_p, double &width_p, double &depth_p, Vector3d &center_p, Quat4d &rot_p):
    height(height_p),width(width_p),depth(depth_p),center(center_p),rot(rot_p),corners(vecvec3d(18))
{
    init();
}

void Package::draw()
{
    glPushMatrix();
    glMultMatrixd(Matrix4d(rot).transpose().ptr());
    glScaled(height,width,depth);
    glTranslated(center[0],center[1],center[2]);

    uint i=0;
    glBegin(GL_QUADS);
    for(;i<8;++i)
        glVertex3dv(corners[i].ptr());
    glEnd();
    glBegin(GL_QUAD_STRIP);
    for(;i<18;++i)
        glVertex3dv(corners[i].ptr());
    glEnd();
    glPopMatrix();
}

void Package::init()
{
    corners[0]=Vector3d(0,0,0);
    corners[1]=Vector3d(1,0,0);
    corners[2]=Vector3d(1,1,0);
    corners[3]=Vector3d(0,1,0);

    corners[4]=Vector3d(0,0,1);
    corners[5]=Vector3d(1,0,1);
    corners[6]=Vector3d(1,1,1);
    corners[7]=Vector3d(0,1,1);

    corners[ 8]=Vector3d(0,1,0);
    corners[ 9]=Vector3d(0,1,1);
    corners[10]=Vector3d(1,1,0);
    corners[11]=Vector3d(1,1,1);
    corners[12]=Vector3d(1,0,0);
    corners[13]=Vector3d(1,0,1);
    corners[14]=Vector3d(0,0,0);
    corners[15]=Vector3d(0,0,1);
    corners[16]=Vector3d(0,1,0);
    corners[17]=Vector3d(0,1,1);
}
