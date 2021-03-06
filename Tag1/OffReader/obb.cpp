#include "obb.h"


AABB::AABB(){}

AABB::AABB(const vecvec3d *p, const vecvecuint ind, Vector3d color_p){
    this->collision=false;
    this->color=color_p;
    this->box_color=Vector3d(0,1,0);
    std::set<Vector3d> point_set;
    for(unsigned int i=0;i<ind.size();i++){
        for(uint j=0;j<ind[i].size();j++){
            point_set.insert(p->at(ind[i][j]));
        }
    }

    this->points=vecvec3d(point_set.size());
    std::set<Vector3d>::iterator beg=point_set.begin();
    std::set<Vector3d>::iterator end=point_set.end();
    for(uint i=0;beg!=end;++i,++beg){
            center=center+*beg;
            points[i]=*beg;
    }

    center=center/points.size();

    for (uint i = 0; i < points.size(); ++i) {
        points[i]=points[i]-center;
    }


    this->rot=Quat4d(0,0,0,1);

    bodycenter=0;
    setAABBCorner(points);
}

AABB::AABB(const vecvec3d &p, Vector3d color_p)
{
    this->collision=false;
    this->color=color_p;
    this->box_color=Vector3d(0,1,0);


    this->points=vecvec3d(p);
    for(uint i=0;i<points.size();++i){
            center=center+points[i];
    }

    center=center/points.size();

    for (uint i = 0; i < points.size(); ++i) {
        points[i]=points[i]-center;
    }

    this->rot=Quat4d(0,0,0,1);

    bodycenter=0;
    setAABBCorner(points);
}

void AABB::setBodyCenter(Vector3d center_b){
    this->bodycenter=center_b;
}

bool AABB::intersectAxis(Vector3d &v,vecvec3d &a, vecvec3d &b,Vector3d &alpha,Vector3d &beta, Vector3d &dc){
    double res=0;
    for (int i = 0; i < 3; ++i) {
        res+=std::abs(alpha[i]*(v.dot(a[i])))+std::abs(beta[i]*(v.dot(b[i])));
    }
    double tmp=v.dot(dc);
    if(std::abs(tmp)>res){
        this->normal=v;
        normal.normalize();
        this->base=-dc/2;
    } else {
        this->normal=0;
        this->base=0;
    }

    return std::abs(v.dot(dc))>res;
}





void AABB::setAABBCorner(const vecvec3d p){
    //double min_x,min_y,min_z,max_x,max_y,max_z;

    R=R.identity();
    axis.push_back(Vector3d(this->R(0,0),this->R(1,0),this->R(2,0)));
    axis.push_back(Vector3d(this->R(0,1),this->R(1,1),this->R(2,1)));
    axis.push_back(Vector3d(this->R(0,2),this->R(1,2),this->R(2,2)));

    Vector3d loc_p;
    max_x=-1e300;
    max_y=-1e300;
    max_z=-1e300;
    min_x= 1e300;
    min_y= 1e300;
    min_z= 1e300;
    for(uint i=0;i<p.size();i++){
        loc_p=p[i];
        debug_points.push_back(loc_p);
        max_x=std::max(max_x,loc_p.x());
        max_y=std::max(max_y,loc_p.y());
        max_z=std::max(max_z,loc_p.z());
        min_x=std::min(min_x,loc_p.x());
        min_y=std::min(min_y,loc_p.y());
        min_z=std::min(min_z,loc_p.z());
    }


    halflength=Vector3d(std::abs((max_x-min_x)/2.),std::abs((max_y-min_y)/2.),std::abs((max_z-min_z)/2.0));
    Vector3d boxCenter=Vector3d(max_x+min_x,max_y+min_y,max_z+min_z)*0.5;
    bodycenter=boxCenter+center;

    min=Vector3d(min_x,min_y,min_z)+center;
    max=Vector3d(max_x,max_y,max_z)+center;

    corner.clear();
    corner.push_back(Vector3d (min_x,max_y,max_z));
    corner.push_back(Vector3d (min_x,min_y,max_z));
    corner.push_back(Vector3d (max_x,max_y,max_z));
    corner.push_back(Vector3d (max_x,min_y,max_z));
    corner.push_back(Vector3d (max_x,max_y,min_z));
    corner.push_back(Vector3d (max_x,min_y,min_z));
    corner.push_back(Vector3d (min_x,max_y,min_z));
    corner.push_back(Vector3d (min_x,min_y,min_z));

    this->rot.set(R);
}


void AABB::caluculateC(const vecvec3d p){
    C=Matrix4d(0.0);
    double vt;
    for(uint i=0;i<p.size();i++){
            C=C+dyadicProdukt(p[i],p[i]);
    }

    int rot=0;
    C.jacobi(eigenvaluesC,R,rot);
}

Matrix4d AABB::dyadicProdukt(Vector3d v1, Vector3d v2){

    Matrix4d result=Matrix4d(v1.x()*v2.x(),v1.x()*v2.y(),v1.x()*v2.z(),0.,
                             v1.y()*v2.x(),v1.y()*v2.y(),v1.y()*v2.z(),0.,
                             v1.z()*v2.x(),v1.z()*v2.y(),v1.z()*v2.z(),0.,
                                 0        ,      0      ,      0      ,1e-4);
    return result;
}

Vector3d AABB::getBodyCenter()
{
    return this->bodycenter;
}

void AABB::draw(){
    glPushMatrix();


    glPointSize(5);
    glColor3d(1,0,1);
    glBegin(GL_POINTS);
    glColor3d(0.5,0.5,0);
    for(uint i =0; i < points.size(); ++i)
    {
        //glVertex3dv(points[i].ptr());
    }
    glColor3d(0.5,0,1);
    glVertex3dv(center.ptr());
    //glVertex3dv(bodycenter.ptr());
    glEnd();

    //glTranslated(center[0], center[1], center[2]);
    //glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_LIGHTING);


    //Matrix4d RT = R.transpose();
    //glMultMatrixd(RT.ptr());

//    glColor3d(.5,1,0);

//    glPushMatrix();
//    glBegin(GL_LINES);
//    //glTranslated(-center[0], -center[1], -center[2]);
//    for (uint i = 0; i < this->axis.size(); ++i) {
//        glVertex3dv((center+bodycenter).ptr());
//        glVertex3dv(((center+bodycenter)+(axis[i])*1500).ptr());
//    }
//    glEnd();
//    glPopMatrix();

    glTranslated(center[0], center[1], center[2]);
    glMultMatrixd(Matrix4d(rot).transpose().ptr());


//    glBegin(GL_POINTS);
//    glColor3d(0.5,0,0);
//    for(uint i =0; i < debug_points.size(); ++i)
//    {
//        glVertex3dv(debug_points[i].ptr());
//    }
//    glEnd();

//    glTranslated(center[0], center[1], center[2]);
//    glColor3d(0,1,1);
//    glBegin(GL_LINES);
//    for (uint i = 0; i < this->axis.size(); ++i) {
//        glVertex3dv(center);
//        glVertex3dv(center+((axis[i])*1500).ptr());
//    }
//    glEnd();

    if(collision)
        glColor3d(0.5,0,0);
    else
        glColor3d(0,0.5,0);

    //glColor3d(box_color.x(),box_color.y(),box_color.z());
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUAD_STRIP);
        for(unsigned int i=0; i<corner.size();i++){
            glVertex3dv(corner[i].ptr());
        }
        glVertex3dv(corner[0].ptr());
        glVertex3dv(corner[1].ptr());
    glEnd();
    glPopMatrix();

}

void AABB::setCollision()
{
    this->collision=true;
}

void AABB::resetCollision()
{
    this->collision=false;
}

bool AABB::intersect(AABB &B){
    bool result=true;

    for(int i=0; i<3;++i){
        result=result && ((min[i]<=B.min[i] && B.min[i]<=max[i]) ||
                          (min[i]<=B.max[i] && B.max[i]<=max[i]) ||
                          (B.min[i]<=min[i] && min[i]<=B.max[i]) ||
                          (B.min[i]<=max[i] && max[i]<=B.max[i]));
    }
    return result;
}

double AABB::getDiameter()
{
    return 2*halflength.length();
}

bool AABB::intersect(Package &A){
    return AABB::intersect(A, *this);
}

bool AABB::intersect(Package &A, AABB &B){

//    OBB bobb=OBB(A.getCorners(),Vector3d(0));
//    bobb.draw();
//    return B.intersect(bobb);

    vecvec3d a,b;
    a.resize(3);b.resize(3);

    for (int i = 0; i < 3; ++i) {
        a[i]=A.rot*A.axis[i];
        b[i]=B.rot*B.axis[i];
    }
    Vector3d alpha=A.halflength*A.zoom_val;
    Vector3d beta=B.halflength;
    Vector3d dc=B.center-A.center;
    //This needs to be the geometric center, not the center of mass
    dc=B.bodycenter-A.center;

    for(int i=0;i<3;i++){
        if(A.intersectAxis(a[i],a,b,alpha,beta,dc))
            return false;
        if(A.intersectAxis(b[i],a,b,alpha,beta,dc))
            return false;
    }
    for(int i=0;i<3;i++){
        for(int k=0;k<3;k++){
            Vector3d co=0;
            co.cross(a[i],b[k]);
            co.normalize();
            if(A.intersectAxis(co,a,b,alpha,beta,dc))
                return false;
        }
    }
    return true;
}


Vector3d AABB::getHalflength()
{
    return halflength;
}

Vector3d AABB::penetration(const vecvec3d T) {
    Vector3d n[13];
    Vector3d c = this->bodycenter;
    Vector3d result;
    double d,dmin,dmax,n2;
    double pen = std::numeric_limits<double>::infinity();

    n[0] = rot*axis[0];
    n[1] = rot*axis[1];
    n[2] = rot*axis[2];
    n[3] = T[0]*T[1];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            n[4+3*i+j] = n[i] % (T[(j+1)%3]-T[j]);

    for(int i=0;i<13;i++) {
        n[i].normalize();
        dmin = dmax = n[i]*(T[0]-c);
        for(int j=1;j<3;j++) {
            d = n[i]*(T[j]-c);
            if (d < dmin) dmin = d; else
            if (d > dmax) dmax = d;
        }
        double s = 0;
        for(int j=0;j<3;j++) {
            double t = n[i]*n[j];
            if (t < 0) t = -t;
            s += halflength[j]*t;
        }

        if ( s < dmin) return 0;
        if (-s > dmax) return 0;
        n2 = n[i]*n[i];
        if (n2 < 1e-6) continue;
        if(pen>(s-dmin)*(s-dmin)/n2){
            pen = (s-dmin)*(s-dmin)/n2;
            result=n[i]*sqrt(pen);
        }
        if(pen>(s+dmin)*(s+dmin)/n2){
            pen = (s+dmin)*(s+dmin)/n2;
            result=n[i]*sqrt(pen);
        }
    }

    return result;
}
