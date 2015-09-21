#include "obb.h"


OBB::OBB(){}

OBB::OBB(vecvec3d *p_p, vecvecuint ind_p, Vector3d color_p){

    this->p=p_p;
    this->ind=ind_p;
    this->color=color_p;
    this->box_color=Vector3d(0,1,0);
    this->zoom_val=1.0;
    //this->center=0;
    //this->box_calculated=false;
    this->q_now=Quat4d(0,0,0,1);
    center=p->at(0);
    for(unsigned int i=1;i<points.size();i++){
        center=center+p->at(i);
    }
    center=center/points.size();
    bodycenter=Vector3d(0,0,0);
    caluculateC();
    setCorner();
}



/*OBB::OBB( OBB& o){
    OBB(o.getP(),o.getInd(),o.getColor());
}*/

void OBB::setBodyCenter(Vector3d center_b){
    this->bodycenter=center_b;
}

bool OBB::intersectAxis(Vector3d &v,vecvec3d &a, vecvec3d &b,Vector3d &alpha,Vector3d &beta, Vector3d &dc){
    double res=0;
    for (int i = 0; i < 3; ++i) {
        res+=std::abs(alpha[i]*(v.dot(a[i])))+std::abs(beta[i]*(v.dot(b[i])));
    }
    double tmp=v.dot(dc);
    if(std::abs(tmp)>res){
        this->normal=v;
        normal.normalize();
        this->base=-dc/zoom_val/2;
    } else {
        this->normal=0;
        this->base=0;
    }

    return std::abs(v.dot(dc))>res;
}

bool OBB::intersect(OBB &B){

    vecvec3d a,b,v;
    a.resize(3);b.resize(3);v.resize(3);

    for (int i = 0; i < 3; ++i) {
        a[i]=q_now*axis[i];
        b[i]=B.q_now*B.axis[i];
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

void OBB::setCorner(){
    //double min_x,min_y,min_z,max_x,max_y,max_z;


    Matrix4d m=Matrix4d();//.transpose();
    m.invert_4x4(R);
    //m=R.transpose();

    Vector3d p=m*points[0];
    max_x=p.x();
    max_y=p.y();
    max_z=p.z();
    min_x=p.x();
    min_y=p.y();
    min_z=p.z();
    for(unsigned int i=0;i<points.size();i++){
        p=m*points[i];
        max_x=std::max(max_x,p.x());
        max_y=std::max(max_y,p.y());
        max_z=std::max(max_z,p.z());
        min_x=std::min(min_x,p.x());
        min_y=std::min(min_y,p.y());
        min_z=std::min(min_z,p.z());
    }
    int row=0;
    if( halflength[1]>halflength[0]&& halflength[1]>halflength[2])
        row=1;
    if( halflength[2]>halflength[0]&& halflength[2]>halflength[1])
        row=2;


    this->longestAxis=Vector3d(R(row,0),R(row,1),R(row,2));
    this->longestAxis.normalize();

    halflength=Vector3d(std::abs((max_x-min_x)/2.),std::abs((max_y-min_y)/2.),std::abs((max_z-min_z)/2.0));
    Vector3d boxCenter=Vector3d(max_x+min_x,max_y+min_y,max_z+min_z);
    boxCenter/=2;
    this->setBodyCenter(boxCenter);

    corner.clear();
    corner.push_back(Vector3d (min_x,max_y,max_z));
    corner.push_back(Vector3d (min_x,min_y,max_z));
    corner.push_back(Vector3d (max_x,max_y,max_z));
    corner.push_back(Vector3d (max_x,min_y,max_z));
    corner.push_back(Vector3d (max_x,max_y,min_z));
    corner.push_back(Vector3d (max_x,min_y,min_z));
    corner.push_back(Vector3d (min_x,max_y,min_z));
    corner.push_back(Vector3d (min_x,min_y,min_z));

    axis.clear();

    axis.push_back(Vector3d(this->R(0,0),this->R(1,0),this->R(2,0)));
    axis.push_back(Vector3d(this->R(0,1),this->R(1,1),this->R(2,1)));
    axis.push_back(Vector3d(this->R(0,2),this->R(1,2),this->R(2,2)));
    for (int i = 0; i < 3; ++i) {
        axis[i].normalize();
    }
    for (uint i = 0; i < corner.size(); ++i) {
        this->corner[i]=R*this->corner[i];
    }

    this->q_now.set(m);
}


void OBB::caluculateC(){
    C=Matrix4d(0.0);
    for(uint i=0;i<points.size();i++){
        C+=dyadicProdukt(points[i],points[i]);
    }
    int rot=0;
    C.jacobi(eigenvaluesC,R,rot);
}

Matrix4d OBB::dyadicProdukt(Vector3d v1, Vector3d v2){
    Matrix4d result=Matrix4d(v1.x()*v2.x(),v1.x()*v2.y(),v1.x()*v2.z(),0.,
                             v1.y()*v2.x(),v1.y()*v2.y(),v1.y()*v2.z(),0.,
                             v1.z()*v2.x(),v1.z()*v2.y(),v1.z()*v2.z(),0.,
                                 0        ,      0      ,      0      ,1e-4);
    return result;
}

void OBB::draw(){
    glPushMatrix();

    glTranslated(-center[0], -center[1], -center[2]);    
    glScaled(zoom_val,zoom_val,zoom_val);
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    //glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_LIGHTING);
    glColor3d(0.5,0,0);

/*
    //debug
    Vector3d a=normal%Vector3d(1,0,0);
    Vector3d b=normal%a;

    vec3dd pi=vec3dd(4);
    double s=8;
    pi[0]=base-a*halflength[0]*s;
    pi[1]=base-b*halflength[1]*s;
    pi[2]=base+a*halflength[0]*s;
    pi[3]=base+b*halflength[1]*s;

    glPointSize(25);
    glBegin(GL_POINTS);
    //glVertex3d(0,0,0);
    glVertex3dv(base.ptr());
    for (int i = 0; i < 3; ++i) {
        Vector3d c=q_now*axis[i];
        c=c*halflength[i];
        glVertex3dv(c.ptr());
        glColor3d(0,0,0);
    }
    glEnd();
*/
    Matrix4d R(q_now);
    Matrix4d RT = R.transpose();
    glMultMatrixd(RT.ptr());

    glColor3d(box_color.x(),box_color.y(),box_color.z());
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUAD_STRIP);
        for(unsigned int i=0; i<corner.size();i++){
            glVertex3dv(corner[i].ptr());
        }
        glVertex3dv(corner[0].ptr());
        glVertex3dv(corner[1].ptr());
    glEnd();

    //glEnable(GL_LIGHTING);
    glPopMatrix();

    /*
    //debug
    glPushMatrix();
    glTranslated(-center[0], -center[1], -center[2]);
    glScaled(zoom_val,zoom_val,zoom_val);
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glColor4d(1,69.0/255,0,0.2);
    glBegin(GL_QUADS);
    for (int i = 0; i < 4; ++i) {
        glVertex3dv(pi[i].ptr());
    }
    glEnd();
    glPopMatrix();*/
}

bool OBB::sortFkt0(const Vector3d &a, const Vector3d &b){ return a[0]<b[0]; }
bool OBB::sortFkt1(const Vector3d &a, const Vector3d &b){ return a[1]<b[1]; }
bool OBB::sortFkt2(const Vector3d &a, const Vector3d &b){ return a[2]<b[2]; }

void OBB::splitOBB(const OBB &A, OBB &A1, OBB &A2){
    vecvec3d *p=new vecvec3d(A.points.size());
    vecvec3d *pA1=new vecvec3d(A.points.size()/2);
    vecvec3d *pA2=new vecvec3d(A.points.size()/2);
    Quat4d q;

    q.set(A.R.inverse(A.R));
    for (uint i=0;i<(A.points).size();i++)
        (*p)[i]=q*(A.points)[i];

    //Find longest Axis
    uint j=0;
    for (int i = 0; i < 3; ++i) {
        if(A.halflength[i]>A.halflength[j])
            j=i;
    }

    //Sort on found axis
    switch (j) {
    case 0:
        std::sort(p->begin(),p->end(),&sortFkt0);
        break;
    case 1:
        std::sort(p->begin(),p->end(),&sortFkt1);
        break;
    case 2:
        std::sort(p->begin(),p->end(),&sortFkt2);
        break;
    default:
        break;
    }

    for(uint i=0;i<pA1->size();i++){
        (*pA1)[i]=(*p)[i];
        (*pA2)[i]=(*p)[i+pA1->size()];
    }
    if(2*pA1->size()<p->size())
        pA2->push_back((*p)[p->size()-1]);
    vecvecuint s;
    s.resize(0);
    A1=OBB(pA1,s,Vector3d());
    A2=OBB(pA2,s,Vector3d());
    A1.q_now=q*A1.q_now;
    A1.center=A.center;
    A1.zoom_val=A.zoom_val;
    A2.q_now=q*A2.q_now;
    A2.center=A.center;
    A2.zoom_val=A.zoom_val;
    A1.caluculateC();
    A1.setCorner();
    A2.caluculateC();
    A2.setCorner();
}
