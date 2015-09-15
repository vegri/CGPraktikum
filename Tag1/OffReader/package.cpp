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

    glColor4f(0.,0.,1.,1.);
    glPointSize(5.);
    glBegin(GL_POINTS);
    for (int i = 0; i < this->d_ray_points.size(); ++i) {
        glVertex3dv(d_ray_points[i].ptr());
    }
    glEnd();

    glBegin(GL_LINES);
    for (int i = 0; i < this->d_ray_lines.size(); ++i) {
        glVertex3dv(d_ray_lines[i].ptr());
    }
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

void Package::getDist(Vector3d loc_origin, Vector3d direction, double &vert_dist, double &parallel_dist)
{
    d_ray_lines.clear();
    d_ray_points.clear();

    //origin of ray in object coordinates
    Vector3d loc=rot*(loc_origin-center);
    Vector3d dir=rot*direction;
    dir.normalize();
    uint i=0;
    Vector3d edge=corners[1]-corners[0];
    //help_layer_normal==direction of distance
    Vector3d help_layer_normal=edge%dir;
    //only if ray and edge are parallel
    if(help_layer_normal.length()==0){
        //insert here decision if 0 or 1 has less distance
        parallel_dist=(corners[0]-loc)*dir;
        vert_dist=(corners[0]-loc-dir*parallel_dist).length();
        return;
    }
    help_layer_normal.normalize();

    double p_dist_c0=(corners[0]-loc)*dir;
    double p_dist_c1=(corners[1]-loc)*dir;

    double v_dist_c0=(corners[0]-loc-dir*p_dist_c0).length();
    double v_dist_c1=(corners[1]-loc-dir*p_dist_c1).length();

    vert_dist=help_layer_normal*(loc-corners[0]);

    Vector3d res=-loc+corners[0]+help_layer_normal*vert_dist;
    double p_dist_ray=(res[1]*edge[0]-res[0]*edge[1])/
                        (dir[1]*edge[0]-dir[0]*edge[1]);
    double p_dist_edge=p_dist_ray*dir[0]/edge[0]-res[0]/edge[0];

    d_ray_points.push_back(corners[0]);
    d_ray_points.push_back(corners[1]);

    //d_ray_lines.push_back(corners[0]);
    //d_ray_lines.push_back(loc+dir*p_dist_c0);

    //std::cout << p_dist_c0 << ":" << p_dist_c1 << std::endl;
    std::cout << p_dist_ray << ":" << p_dist_edge << std::endl;

    //d_ray_lines.push_back(corners[1]);
    //d_ray_lines.push_back(loc+dir*p_dist_c1);

    d_ray_lines.push_back(corners[0]+edge*p_dist_edge);
    d_ray_lines.push_back(corners[0]+edge*p_dist_edge+help_layer_normal*vert_dist);

    d_ray_lines.push_back(corners[0]+edge*p_dist_edge);
    d_ray_lines.push_back(corners[0]);

    d_ray_lines.push_back(loc+dir*p_dist_ray);
    d_ray_lines.push_back(loc);

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

    center[0]+=height/2;
    center[1]+=width/2;
    center[2]+=depth/2;


    for (uint i = 0; i < 18; ++i) {
        corners[i]-=center;
    }

    resetColor();
    serial=next_serial;
    ++next_serial;
}
