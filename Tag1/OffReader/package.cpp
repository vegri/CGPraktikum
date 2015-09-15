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

    if(move_in_dir!=-1){
        GLdouble *oldWidth;
        glGetDoublev(GL_LINE_WIDTH,oldWidth);
        glLineWidth(4.5);
        glColor4d(0,1,0,1);
        glBegin(GL_LINES);
        glVertex3dv(corners[move_in_dir].ptr());
        glVertex3dv((corners[move_in_dir]+move_dir).ptr());
        glEnd();
        glLineWidth(*oldWidth);
    }

    glColor4f(0.,0.,1.,1.);
    glPointSize(5.);
    glBegin(GL_POINTS);
    for (uint i = 0; i < this->d_ray_points.size(); ++i) {
        glVertex3dv(d_ray_points[i].ptr());
    }
    glEnd();

    glBegin(GL_LINES);
    for (uint i = 0; i < this->d_ray_lines.size(); ++i) {
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
    if(move_dir_b){
        this->center+=move_dir*(move_p*move_dir/move_dir.lengthSquared());
    } else
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

void Package::getDist(Vector3d loc_origin, Vector3d direction, double epsilon, double &vert_dist, double &parallel_dist)
{
    //DEBUG
    d_ray_lines.clear();
    d_ray_points.clear();
    write_lines=false;
    //DEBUG END
    uint g_i,g_j;

    std::vector<uint> idx_set(24);
    idx_set[0]=0; idx_set[1]=1; idx_set[2]=1; idx_set[3]=2;
    idx_set[4]=2; idx_set[5]=3; idx_set[6]=3; idx_set[7]=0;

    idx_set[8]=4; idx_set[9]=5; idx_set[10]=5; idx_set[11]=6;
    idx_set[12]=6; idx_set[13]=7; idx_set[14]=7; idx_set[15]=4;

    idx_set[16]=0; idx_set[17]=5; idx_set[18]=1; idx_set[19]=6;
    idx_set[20]=2; idx_set[21]=7; idx_set[22]=3; idx_set[23]=4;

    vert_dist=1e300;
    parallel_dist=1e300;
    double act_vert_dist,act_par_dist;

    for(uint i=0;i<23;i+=2){
        getDistEdge(loc_origin,direction,idx_set[i],idx_set[i+1],act_vert_dist,act_par_dist);
        if(fabs(act_vert_dist)<epsilon && fabs(act_par_dist)<parallel_dist){
            vert_dist=fabs(act_vert_dist);
            parallel_dist=fabs(act_par_dist);
            this->move_in_dir=idx_set[i];
            this->move_dir=-corners[idx_set[i]]+corners[idx_set[i+1]];
            //DEBUG
            g_i=idx_set[i]; g_j=idx_set[i+1];
        }
        //DEBUG
        if(false){
            d_ray_lines.push_back(corners[idx_set[i]]);
            d_ray_lines.push_back(corners[idx_set[i+1]]);
        }
    }

    //DEBUG
    if(vert_dist<1e299){
        write_lines=true;
        getDistEdge(loc_origin,direction,g_i,g_j,act_vert_dist,act_par_dist);
        write_lines=false;
    }
}

void Package::getDistEdge(Vector3d loc_origin, Vector3d direction, uint i, uint j, double &vert_dist, double &parallel_dist)
{
    //origin of ray in object coordinates
    Vector3d loc=rot*(loc_origin-center);
    Vector3d dir=rot*direction;
    dir.normalize();

    Vector3d edge=corners[j]-corners[i];
    //help_layer_normal==direction of distance
    Vector3d help_layer_normal=edge%dir;
    //only if ray and edge are parallel
    if(help_layer_normal.length()==0){
        //insert here decision if 0 or 1 has less distance
        parallel_dist=(corners[i]-loc)*dir;
        vert_dist=(corners[i]-loc-dir*parallel_dist).length();
        return;
    }
    help_layer_normal.normalize();

    double p_dist_c0=(corners[i]-loc)*dir;
    double p_dist_c1=(corners[j]-loc)*dir;

    double v_dist_c0=(corners[i]-loc-dir*p_dist_c0).length();
    double v_dist_c1=(corners[j]-loc-dir*p_dist_c1).length();

    vert_dist=help_layer_normal*(loc-corners[i]);

    Vector3d res=-loc+corners[i]+help_layer_normal*vert_dist;

    //Select right equations for resolution
    uint k=1;
    //Proove if two times same equation selected||prrove if division by 0 is happening in computation
    //If one appears test next k
    //use equation idx_1=k%3 and idx_0=k/3
    for(;k%3==k/3 || dir[k%3]*edge[k/3]-dir[k/3]*edge[k%3]==0 || edge[k/3]==0;++k){}

    double p_dist_ray=(res[k%3]*edge[k/3]-res[k/3]*edge[k%3])/ //mu
                        (dir[k%3]*edge[k/3]-dir[k/3]*edge[k%3]);
    double p_dist_edge=p_dist_ray*dir[k/3]/edge[k/3]-res[k/3]/edge[k/3]; //lambda

    //DEBUG
    if(write_lines){
        d_ray_points.push_back(corners[i]);
        d_ray_points.push_back(corners[j]);

        d_ray_lines.push_back(corners[i]);
        d_ray_lines.push_back(loc+dir*p_dist_c0);

        d_ray_lines.push_back(corners[j]);
        d_ray_lines.push_back(loc+dir*p_dist_c1);

        d_ray_lines.push_back(corners[i]+edge*p_dist_edge);
        d_ray_lines.push_back(corners[i]+edge*p_dist_edge+
                              help_layer_normal*(help_layer_normal*(loc-corners[i])));

        d_ray_lines.push_back(corners[i]+edge*p_dist_edge);
        d_ray_lines.push_back(corners[i]);

        d_ray_lines.push_back(loc+dir*p_dist_ray);
        d_ray_lines.push_back(loc);
    }
    //DEBUG END


    //select the best result
    //If min on edge
    if(!(0>p_dist_edge || 1<p_dist_edge)){
        parallel_dist=p_dist_ray;
        if(p_dist_ray<0){
            vert_dist=1e300;
        } else {
            vert_dist=fabs(vert_dist);
        }
    //If min on second corner
    } else if(fabs(v_dist_c0)>fabs(v_dist_c1)){
        parallel_dist=p_dist_c1;
        vert_dist=fabs(v_dist_c1);
    //If min on first corner
    } else {
        parallel_dist=p_dist_c0;
        vert_dist=fabs(v_dist_c0);
    }
}

void Package::setMoveDir(bool move_dir_p)
{
    this->move_dir_b=move_dir_p;
}

void Package::init()
{
    corners[ 0]=Vector3d(0,0,0);
    corners[ 1]=Vector3d(height,0,0);
    corners[ 2]=Vector3d(height,width,0);
    corners[ 3]=Vector3d(0,width,0);

    corners[ 4]=Vector3d(0,width,depth);
    corners[ 5]=Vector3d(0,0,depth);
    corners[ 6]=Vector3d(height,0,depth);
    corners[ 7]=Vector3d(height,width,depth);


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
    move_in_dir=-1;
}
