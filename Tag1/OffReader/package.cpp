#include "package.h"

uint Package::next_serial=1;
Vector4d Package::pickColor=Vector4d(0,1,1,0.8);
Vector4d Package::pickColorColl=Vector4d(1,0.4,0,0.8);

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

    glTranslated(center[0],center[1],center[2]);
    glMultMatrixd(Matrix4d(rot).transpose().ptr());
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

    if(collision){
        if(picked)
            setColor(pickColorColl);
        else
            setColor(Vector4d(1,0,0,0.5));
    }
    else{
        if(picked)
            setColor(pickColor);
        else
            resetColor();
    }

    if(picked){
        GLUquadricObj *quadric;
        quadric = gluNewQuadric();
        glPushMatrix();
        gluQuadricDrawStyle(quadric, GLU_LINE);
        //X-Y-Layer
        if(this->rot_dir==2)
            glColor4d(1,1,0,1);
        else
            glColor4d(0,1,0,1);
        gluDisk(quadric,  circle_rad*0.999,circle_rad*1.01,  40, 5);
        //Y-Z-Layer
        if(this->rot_dir==1)
            glColor4d(1,1,0,1);
        else
            glColor4d(0,0,1,1);
        glMultMatrixd(Matrix4d(Quat4d(3.141592653589793/2,Vector3d(1,0,0))).transpose().ptr());
        gluDisk(quadric,  circle_rad*0.999,circle_rad*1.01,  40, 5);
        //X-Z-Layer
        if(this->rot_dir==0)
            glColor4d(1,1,0,1);
        else
            glColor4d(1,0,1,1);
        glMultMatrixd(Matrix4d(Quat4d(-3.141592653589793/2,Vector3d(0,1,0))).transpose().ptr());
        gluDisk(quadric,  circle_rad*0.999,circle_rad*1.01,  40, 5);
        glPopMatrix();
    }
    if(picked){
        GLUquadricObj *quadric;
        quadric = gluNewQuadric();

        glColor4d(.1,.1,.1,0.1);
        gluQuadricDrawStyle(quadric, GLU_LINE);
        gluSphere( quadric , this->getDiameter()/2,40,40);
    }

    if(move_dir_b){
        glLineWidth(4.5);
        glColor4d(0,1,0,1);
        glBegin(GL_LINES);
        glVertex3dv(corners[move_in_dir].ptr());
        glVertex3dv((corners[move_in_dir]+move_dir).ptr());
        glEnd();
        glLineWidth(2.0);
    }

    glColor4f(0.,0.,1.,1.);
    glPointSize(8.);
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
    color[3]=0.5;
}

void Package::setColor(Vector4d color_p)
{
    this->color=color_p;
}

void Package::move(Vector3d move_p)
{
    if(move_dir_b){
        //pack_center=center+hit
        //move_p=u+dp*(dp*(pack_center-u))-pack_center
        this->center+=rot*move_dir*(move_p*(rot*move_dir)/move_dir.lengthSquared());
    } else
        this->center+=move_p;
}

void Package::setRot(Quat4d rot_p)
{
    this->rot=rot_p;
}

void Package::rotate(Quat4d rot_p)
{
    this->rot=rot_p*this->rot;
    this->rot.normalize();
}

void Package::markRot()
{
    this->rot_old=rot;
}

void Package::rotateMarked(Quat4d rot_p)
{
    this->rot=rot_p*rot_old;
}

double Package::getDiameter()
{
    return sqrt(height*height+width*width+depth*depth);
}

Vector3d Package::getCenter()
{
    return center;
}

void Package::pick(bool picked)
{
    this->picked=picked;

    rot_dir=-1;
}

Vector3d Package::getRotProjection()
{
    Vector3d v=Vector3d(0);
    v[rot_dir]=1;
    v=rot*v;
    return v;
}

void Package::removeRot()
{
    this->rot_dir=-1;
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
    if(false){
        write_lines=true;
        getDistEdge(loc_origin,direction,g_i,g_j,act_vert_dist,act_par_dist);
        write_lines=false;
    }
}

void Package::getDistEdge(Vector3d loc_origin, Vector3d direction, uint i, uint j, double &vert_dist, double &parallel_dist)
{
    //origin of ray in object coordinates
    Vector3d loc=rot.inverse()*(loc_origin-center);
    Vector3d dir=rot.inverse()*direction;
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

bool Package::getHit(Vector3d loc_origin, Vector3d direction, Vector3d &hit, double &parallel_dist)
{
    Vector3d loc=rot.inverse()*(loc_origin-center);
    Vector3d dir=rot.inverse()*direction;
    dir.normalize();
    bool result=false;

    std::vector<uint> idx_set(18);
    idx_set[ 0]=0; idx_set[ 1]=1; idx_set[ 2]=3;
    idx_set[ 3]=0; idx_set[ 4]=1; idx_set[ 5]=5;
    idx_set[ 6]=1; idx_set[ 7]=6; idx_set[ 8]=2;
    idx_set[ 9]=6; idx_set[10]=5; idx_set[11]=7;
    idx_set[12]=5; idx_set[13]=0; idx_set[14]=4;
    idx_set[15]=4; idx_set[16]=3; idx_set[17]=7;

    parallel_dist=1e300;
    for (uint i = 0; i < 18; i+=3) {
        Vector3d foot=corners[idx_set[i]];
        Vector3d plane_vec1=corners[idx_set[i+1]]-corners[idx_set[i]];
        Vector3d plane_vec2=corners[idx_set[i+2]]-corners[idx_set[i]];

        double mu,lam1,lam2;
        getIntersectionLinePlane(loc,  dir, foot, plane_vec1, plane_vec2, mu, lam1, lam2);
        if(0<lam1 && lam1<1 && 0<lam2 && lam2<1 && mu < parallel_dist){
            hit=foot+plane_vec1*lam1+plane_vec2*lam2;
            parallel_dist=mu;
            result=true;
        }

        //DEBUG
        if(false){
            d_ray_lines.push_back(foot);
            d_ray_lines.push_back(foot+plane_vec1+plane_vec2);

            d_ray_points.push_back(foot+plane_vec1*lam1+plane_vec2*lam2);
            d_ray_points.push_back(loc+dir*mu);
            d_ray_points.push_back(foot+plane_vec1);
            d_ray_points.push_back(foot);
        }
        //DEBUG END
    }

    //DEBUG
    if(result){
        d_ray_points.push_back(hit);
        //d_ray_lines.push_back(loc+dir*parallel_dist);
        //d_ray_lines.push_back(loc);
    }
    //DEBUG END

    hit=rot*hit+center;

    return result;

}


//Javadoc
void Package::getDistCircleLine(Vector3d loc_origin, Vector3d direction, double epsilon, double &vert_dist, double &parallel_dist, Vector3d &hit)
{
    //DEBUG
    d_ray_lines.clear();
    d_ray_points.clear();
    write_lines=false;
    //DEBUG END
    Vector3d loc=rot.inverse()*(loc_origin-center);
    Vector3d dir=rot.inverse()*direction;
    dir.normalize();


    uint i=0,j=1,k=2;
    rot_dir=-1;
    parallel_dist=1e300;

    for (;i<3;++i) {
        j=(i+1)%3;
        k=(i+2)%3;

        //perpendicular=perp=senkrecht to circle
        //plane normal
        Vector3d perp_circ=Vector3d(0);
        perp_circ[k]=1;
        //plane vectors
        Vector3d para_dir =Vector3d(0);
        para_dir[i]=1;
        Vector3d perp_dir =Vector3d(0);
        perp_dir[j]=1;

        double mu,lam1,lam2;

        //cap and bottom
        int l=-1;
        for(;l<2;l+=2){
            getIntersectionLinePlane(loc,  dir,  Vector3d(0,0,l*epsilon) , para_dir, perp_dir, mu, lam1, lam2);
            hit=loc+dir*mu;
            if(hit.length()<this->circle_rad+epsilon && hit.length()>this->circle_rad-epsilon && parallel_dist>fabs(mu)){
                parallel_dist=fabs(mu);
                vert_dist=fabs(hit.length()-this->circle_rad);
                d_ray_points.push_back(hit);
                hit=rot*hit+center;
                rot_dir=k;
                l=4;
            }
        }
        if(l==4 || l==6){
            continue;
        }

        //sides
        //solves loc+parallel_dist*dir=vert_dist*(dir%perp_circ)
        double t=dir[k];
        dir[k]=0;
        perp_dir=dir%perp_circ;
        perp_dir.normalize();
        dir[k]=t;

        double v_dist_loc,p_dist_loc;
        p_dist_loc=(loc[j]*perp_dir[i]-loc[i]*perp_dir[j])/ //mu
                        (dir[j]*perp_dir[i]-dir[i]*perp_dir[j]);
        if(perp_dir[j]!=0){
            v_dist_loc=-p_dist_loc*dir[j]/perp_dir[j]-loc[j]/perp_dir[j]; //lambda
        } else if(perp_dir[i]!=0){
            v_dist_loc=p_dist_loc*dir[i]/perp_dir[i]-loc[i]/perp_dir[i]; //lambda
        } else {
            std::cout << "Strange things happening, package.cpp getDistCircleLine()" << std::endl;
            return;
        }


        if(fabs(v_dist_loc)<this->circle_rad+epsilon){
            t=sqrt((circle_rad+epsilon)*(circle_rad+epsilon)-v_dist_loc*v_dist_loc);
            if(p_dist_loc-t<0)
                t=-t;
            p_dist_loc=p_dist_loc-t;

            if(fabs((loc-dir*p_dist_loc)[k])<epsilon && fabs(p_dist_loc)<parallel_dist){
                hit=loc-dir*p_dist_loc;
                d_ray_points.push_back(hit);
                vert_dist=fabs(v_dist_loc);
                parallel_dist=fabs(p_dist_loc);
                rot_dir=k;
            }
        }
    }
}

double Package::getCircleRad()
{
    return circle_rad;
}

//Solves loc+mu*dir=foot+lam1*vec1+lam2*vec2
void Package::getIntersectionLinePlane(const Vector3d &loc,  const Vector3d &dir,
                                       const Vector3d &foot, const Vector3d &plane_vec1, const Vector3d &plane_vec2,
                                       double &mu, double &lam1, double &lam2){
    Vector3d res;
    Matrix4d mat;
    mat.makeIdentity();
    for(uint i=0;i<3;++i){
        mat(i,0)=-dir[i];
        mat(i,1)=-plane_vec1[i];
        mat(i,2)=plane_vec2[i];
    }

    solve3dLinearSystem(mat,res,loc-foot);
    mu=res[0];
    lam1=res[1];
    lam2=res[2];

}

void Package::solve3dLinearSystem(const Matrix4d &m, Vector3d &x, const Vector3d &s)
{
    double a=m(0,0),b=m(0,1),c=m(0,2),
           d=m(1,0),e=m(1,1),f=m(1,2),
           g=m(2,0),h=m(2,1),i=m(2,2),
           j=s[0],k=s[1],l=s[2];
    double r,det;
    x=Vector3d();
    det=a*(e*i - f*h) + b*(f*g - d*i) + c*(d*h - e*g);

    r=b*(f*l - i*k) + c*(h*k - e*l) + j*(e*i - f*h);
    x[0]=r/det;
    r=a*(f*l - i*k) + c*(g*k - d*l) + j*(d*i - f*g);
    x[1]=r/det;
    r=a*(e*l - h*k) + b*(g*k - d*l) + j*(d*h - e*g);
    x[2]=r/det;
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

    halflength=center;

    axis.resize(3);
    axis[0]=Vector3d(1,0,0);
    axis[1]=Vector3d(0,1,0);
    axis[2]=Vector3d(0,0,1);

    for (uint i = 0; i < 18; ++i) {
        corners[i]-=center;
    }
    do{
        serial=next_serial;
        ++next_serial;
    }
    while( (serial/9%3)/3.0>0.5 && (serial%3)/3.0>0.6);
    resetColor();
    move_in_dir=-1;
    rot_dir=0;
    move_dir_b=false;
    rot_dir_b=false;
    rot_ball_b=false;
    rot_dir=-1;
    circle_rad=sqrt(fmax(fmax(width*width+height*height,height*height+depth*depth),depth*depth+width*width))/2;
}

bool Package::isPicked()
{
    return picked;
}

void Package::setCollision(Package &other)
{
    this->collision=true;
}

void Package::resetCollision()
{
    this->collision=false;
}
//collision calculation
bool Package::intersectAxis(Vector3d &v, vecvec3d &a, vecvec3d &b, Vector3d &alpha, Vector3d &beta, Vector3d &dc){
    double res=0;
    for (int i = 0; i < 3; ++i) {
        res+=std::abs(alpha[i]*(v.dot(a[i])))+std::abs(beta[i]*(v.dot(b[i])));
    }

    if(std::abs(v.dot(dc))<res)
        this->collDir=v.normalized()*(std::abs(v.dot(dc))-res);
    std::cout << res<<  std::endl;

    return std::abs(v.dot(dc))>res;
}

bool Package::resolveCollision(Package &B){
    this->collDir=0;

    srand(time(NULL));
    //Vector3d epsilon=Vector3d(random()*ULONG_MAX,random()*ULONG_MAX,random()*ULONG_MAX).normalized()*0.0001*B.getDiameter();
    //B.center+=epsilon;
    bool intersectionOccured=intersect(B);
    //if(!intersectionOccured)
      //  B.center-=epsilon;


    if((this->center-B.center-collDir).lengthSquared()<(this->center-B.center).lengthSquared())
        collDir=-collDir;
    this->center-=collDir*1.01/2;
    B.center+=collDir*1.01/2;
    this->collDir=0;
    B.collDir=0;
    return intersectionOccured;
}

bool Package::intersect(Package &B){

    vecvec3d a,b,v;

    a.resize(3);b.resize(3);v.resize(3);

    for (int i = 0; i < 3; ++i) {
        a[i]=rot*axis[i];
        b[i]=B.rot*B.axis[i];
    }
    Vector3d alpha=halflength*zoom_val;
    Vector3d beta=B.halflength*B.zoom_val;
    Vector3d dc=B.center-center;

    for(int i=0;i<3;i++){
        if(this->intersectAxis(a[i],a,b,alpha,beta,dc))
            return false;
        if(this->intersectAxis(b[i],a,b,alpha,beta,dc))
            return false;
    }
    for(int i=0;i<3;i++){
        for(int k=0;k<3;k++){
            Vector3d co=0;
            co.cross(a[i],b[k]);
            co.normalize();
            if(this->intersectAxis(co,a,b,alpha,beta,dc))
                return false;
        }
    }
    return true;

}
