#include "package.h"

uint Package::next_serial=0;

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
    glTranslated(center[0],center[1],center[2]);
    glColor4dv(color.ptr());
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

void Package::resetColor()
{
    color[0]=(serial%3)/3.0;
    color[1]=(serial/3%3)/3.0;
    color[2]=(serial/9%3)/3.0;
    color[3]=0.8;
}

void Package::setColor(Vector4d color_p)
{
    this->color=color_p;
}

void Package::move(Vector3d move_p)
{
    this->center+=move_p;
}

double Package::getDiameter()
{
    return sqrt(height*height+width*width+depth*depth);
}

Vector3d Package::getCenter()
{
    return center;
}

void Package::init()
{
    corners[0]=Vector3d(0,0,0);
    corners[1]=Vector3d(height,0,0);
    corners[2]=Vector3d(height,width,0);
    corners[3]=Vector3d(0,width,0);

    corners[4]=Vector3d(0,0,depth);
    corners[5]=Vector3d(height,0,depth);
    corners[6]=Vector3d(height,width,depth);
    corners[7]=Vector3d(0,width,depth);

    corners[ 8]=Vector3d(0,width,0);
    corners[ 9]=Vector3d(0,width,depth);
    corners[10]=Vector3d(height,width,0);
    corners[11]=Vector3d(height,width,depth);
    corners[12]=Vector3d(height,0,0);
    corners[13]=Vector3d(height,0,depth);
    corners[14]=Vector3d(0,0,0);
    corners[15]=Vector3d(0,0,depth);
    corners[16]=Vector3d(0,width,0);
    corners[17]=Vector3d(0,width,depth);

    center[0]=height/2;
    center[1]=width/2;
    center[2]=depth/2;


    for (uint i = 0; i < 18; ++i) {
        corners[i]-=center;
    }

    resetColor();
    serial=next_serial;
    ++next_serial;
}
