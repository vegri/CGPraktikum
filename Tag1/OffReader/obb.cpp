#include "obb.h"


OBB::OBB(){}

OBB::OBB(const vecvec3d *p, const vecvecuint ind, Vector3d color_p){
    this->collision=false;
    this->color=color_p;
    this->box_color=Vector3d(0,1,0);
    this->points=vecvec3d(ind.size()*3);

    center=0;
    for(unsigned int i=0;i<ind.size();i++){
        for(uint j=0;j<ind[i].size();j++){
            center=center+p->at(ind[i][j]);
            points[i*3+j]=p->at(ind[i][j]);
        }
    }
    center=center/(ind.size()*3);

    this->rot=Quat4d(0,0,0,1);

    bodycenter=0;
    setCorner(p,ind);
}

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
        this->base=-dc/2;
    } else {
        this->normal=0;
        this->base=0;
    }

    return std::abs(v.dot(dc))>res;
}





void OBB::setCorner(const vecvec3d *p, const vecvecuint ind){
    //double min_x,min_y,min_z,max_x,max_y,max_z;
    caluculateC(p,ind);

    Matrix4d C=Matrix4d();
    Vector3d r;

    for(uint i=0;i<ind.size()*3;i++) {
      r = (*p)[ind[i/3][i%3]] - center;
      C(0,0) += r[1]*r[1]+r[2]*r[2];
      C(1,1) += r[0]*r[0]+r[2]*r[2];
      C(2,2) += r[0]*r[0]+r[1]*r[1];
      C(0,1) -= r[0]*r[1];
      C(0,2) -= r[0]*r[2];
      C(1,2) -= r[1]*r[2];
    }

    C(1,0) = C(0,1);  //nutze Symmetrie der Covarianzmatrix
    C(2,0) = C(0,2);
    C(2,1) = C(1,2);

    C(0,3) = C(3,0) = 0.0;
    C(1,3) = C(3,1) = 0.0;
    C(2,3) = C(3,2) = 0.0;
    C(3,3) = 1.0;

    //berechne Eigenwerte und Eigenvektoren
    //int nrot;
    //C.jacobi(eigenvaluesC,R,nrot);

//    for (uint i = 0; i < 3; ++i) {
//        for (uint j = 0; j < 3; ++j) {
//            std::cout << C(i,j)-this->C(i,j) << " ; ";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;

    Matrix4d m=Matrix4d();
    m=R.transpose();

    Vector3d loc_p;
    max_x=-1e300;
    max_y=-1e300;
    max_z=-1e300;
    min_x= 1e300;
    min_y= 1e300;
    min_z= 1e300;
    for(unsigned int i=0;i<ind.size();i++){
        for(uint j=0;j<ind[i].size();j++){
            loc_p=m*(p->at(ind[i][j])-center);
            max_x=std::max(max_x,loc_p.x());
            max_y=std::max(max_y,loc_p.y());
            max_z=std::max(max_z,loc_p.z());
            min_x=std::min(min_x,loc_p.x());
            min_y=std::min(min_y,loc_p.y());
            min_z=std::min(min_z,loc_p.z());
        }
    }


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

//    axis.push_back(Vector3d(this->R(0,0),this->R(1,0),this->R(2,0)));
//    axis.push_back(Vector3d(this->R(0,1),this->R(1,1),this->R(2,1)));
//    axis.push_back(Vector3d(this->R(0,2),this->R(1,2),this->R(2,2)));
    //axis.push_back(Vector3d(this->R(0,0),this->R(0,1),this->R(0,2)));
    //axis.push_back(Vector3d(this->R(1,0),this->R(1,1),this->R(1,2)));
    //axis.push_back(Vector3d(this->R(2,0),this->R(2,1),this->R(2,2)));
    axis.push_back(Vector3d(0,0,1));
    axis.push_back(Vector3d(0,1,0));
    axis.push_back(Vector3d(1,0,0));
    for (int i = 0; i < 3; ++i) {
        axis[i].normalize();
    }

    this->rot.set(R);
}


void OBB::caluculateC(const vecvec3d *p, const vecvecuint ind){
    C=Matrix4d(0.0);
    double vt;
    for(uint i=0;i<ind.size();i++){
        for(uint j=0;j<3;j++){
            C+=dyadicProdukt((*p)[ind[i][j]]-center,(*p)[ind[i][j]]-center)*(-1);
            vt=((*p)[ind[i][j]]-center)*((*p)[ind[i][j]]-center);
            C(0,0)+=vt; C(1,1)+=vt; C(2,2)+=vt;
        }
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

    glPointSize(5);
    glColor3d(1,0,1);
    glBegin(GL_POINTS);
    for(uint i =0; i < points.size(); ++i)
    {
        glVertex3dv(points[i].ptr());
    }
    glEnd();

    glTranslated(center[0], center[1], center[2]);
    //glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_LIGHTING);


    //Matrix4d RT = R.transpose();
    //glMultMatrixd(RT.ptr());

//    glColor3d(.5,1,0);

//    glBegin(GL_LINES);
//    for (uint i = 0; i < this->axis.size(); ++i) {
//        glVertex3dv(Vector3d(0).ptr());
//        glVertex3dv(((axis[i])*1500).ptr());
//    }
//    glEnd();

    glMultMatrixd(Matrix4d(rot).transpose().ptr());

    glColor3d(0,1,1);
    glBegin(GL_LINES);
    for (uint i = 0; i < this->axis.size(); ++i) {
        glVertex3dv(Vector3d(0).ptr());
        glVertex3dv(((axis[i])*1500).ptr());
    }
    glEnd();

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

    //glEnable(GL_LIGHTING);
    glPopMatrix();

}

void OBB::setCollision()
{
    this->collision=true;
}

void OBB::resetCollision()
{
    this->collision=false;
}

bool OBB::intersect(OBB &B){

    vecvec3d a,b,v;

    a.resize(3);b.resize(3);v.resize(3);

    for (int i = 0; i < 3; ++i) {
        a[i]=rot*axis[i];
        b[i]=B.rot*B.axis[i];
    }
    Vector3d alpha=halflength;
    Vector3d beta=B.halflength;
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

bool OBB::intersect(Package &A){
    return OBB::intersect(A, *this);
}

bool OBB::intersect(Package &A, OBB &B){

    vecvec3d a,b;
    a.resize(3);b.resize(3);

    for (int i = 0; i < 3; ++i) {
        a[i]=A.rot*A.axis[i];
        b[i]=B.rot*B.axis[i];
    }
    Vector3d alpha=A.halflength*A.zoom_val;
    Vector3d beta=B.halflength;
    Vector3d dc=B.center-A.center;

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
