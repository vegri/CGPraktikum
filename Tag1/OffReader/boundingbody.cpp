#include "boundingbody.h"

BoundingBody::BoundingBody()
{
}

void BoundingBody::draw_mesh(){
    if(this->collision.size()>0)
        draw_mesh_collision();
    else
        draw_mesh_no_collision();
}

void BoundingBody::draw_mesh_collision(){

    Vector3d n;

    glPushMatrix();
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glTranslated(-center[0], -center[1], -center[2]);
    Matrix4d R(q_now);
    Matrix4d RT = R.transpose();
    glMultMatrixd(RT.ptr());
    glScaled(zoom_val,zoom_val,zoom_val);

    //glPopMatrix();

    vec3dd a(3),b(3);
    bool found=false;
    //glPointSize(5);
    //glBegin(GL_POINTS);
    for(unsigned int j=0;j<ind.size();j+=3) {
        for (uint k = 0; k < this->collision.size(); ++k) {
            BoundingBody* B=this->collision[k];
            for (uint l = 0; l < B->ind.size(); l+=3) {
                for (uint i = 0; i < 3; ++i) {
                    a[i]=(q_now*p[ind[j+i]]*zoom_val)-center;
                    b[i]=(B->q_now*B->p[B->ind[l+i]]*B->zoom_val)-B->center;
                    //glVertex3dv(b[i].ptr());
                }

                if(intersectTrianlge(a,b)){
                    k=this->collision.size();
                    l=B->ind.size();
                    found=true;
                }
            }

        }



        if(found)
            glColor4d(collision_color[0],collision_color[1],collision_color[2],0.7);
        else
            glColor4d(color[0],color[1],color[2],0.7);

        glBegin(GL_TRIANGLES);
        n.cross((p[ind[j+1]]-p[ind[j]]),(p[ind[j+2]]-p[ind[j]]));
        n.normalize(n);
        glNormal3dv(n.ptr());
        glVertex3dv(p[ind[j]].ptr());
        glVertex3dv(p[ind[j+1]].ptr());
        glVertex3dv(p[ind[j+2]].ptr());
        glEnd();
        found=false;
    }
    //glEnd();

    glPopMatrix();
}



void BoundingBody::draw_mesh_no_collision()
{
    glColor4d(color[0],color[1],color[2],0.7);
    Vector3d n;

    glPushMatrix();
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glTranslated(-center[0], -center[1], -center[2]);
    Matrix4d R(q_now);
    Matrix4d RT = R.transpose();
    glMultMatrixd(RT.ptr());
    glScaled(zoom_val,zoom_val,zoom_val);

    glBegin(GL_TRIANGLES);
    for(unsigned int j=0;j<ind.size();j+=3) {
        n.cross((p[ind[j+1]]-p[ind[j]]),(p[ind[j+2]]-p[ind[j]]));
        n.normalize(n);
        glNormal3dv(n.ptr());
        glVertex3dv(p[ind[j]].ptr());
        glVertex3dv(p[ind[j+1]].ptr());
        glVertex3dv(p[ind[j+2]].ptr());
    }
    glEnd();
    glPopMatrix();
}

bool BoundingBody::intersectTrianlge(vec3dd &a_p, vec3dd &b_p)
{
    vec3dd a(3),b(3);
    Vector3d ca,cb,na,nb;

    for (int i = 0; i < 3; ++i) {
        ca+=a_p[i];
        cb+=b_p[i];
    }
    ca/=3; cb/=3;

    double maxLenA=0;
    double maxLenB=0;

    for (int i = 0; i < 3; ++i) {
        a[i]=a_p[i]-ca;
        b[i]=b_p[i]-cb;
        maxLenA=std::max(maxLenA,a[i].lengthSquared());
        maxLenB=std::max(maxLenB,b[i].lengthSquared());
    }
    if((ca-cb).lengthSquared()>2*(maxLenA+maxLenB)){
        return false;
    }

    na.cross(a[0],a[1]); nb.cross(b[0],b[1]);

    if (planeTest(na,a,b,ca-cb))
        return false;
    if (planeTest(nb,a,b,ca-cb))
        return false;
    for (int i = 0; i < 3; ++i) {
        for (int j = i+1; j < 3; ++j) {
            if (i==j)
                continue;
            for (int k = 0; k < 3; ++k) {
                for (int l = k+1; l < 3; ++l) {
                    if (k==l)
                        continue;
                    Vector3d ba=(a[i]-a[j]);
                    ba.normalize();
                    ba=a[i]-ba*ba.dot(a[i]);

                    Vector3d bc=(a[i]-a[j])%(b[k]-b[l]);
                    double val=bc*(ca+ba);
                    int signA=0;
                    int signB=0;
                    bool found=true;

                    for (int m = 0; m < 3; ++m) {
                        if(m!=i && m!=j)
                            signA=bc.dot(ca+a[m])-val<0 ? -1 : 1;
                    }

                    for (int m = 0; m < 3; ++m) {
                        signB=bc.dot(cb+b[m])-val<0 ? -1 : 1;
                        found=found && (signA!=signB);
                    }

                    if (found)
                        return false;
                }
            }
        }
    }

    return true;
}

bool BoundingBody::planeTest(const Vector3d &v, const vec3dd &a, const vec3dd &b, const Vector3d &dc)
{
    double res1=0,res2=0,res=0;
    for (int i = 0; i < 3; ++i) {
        res1=std::max(res1,std::abs(v.dot(a[i])));
        res2=std::max(res2,std::abs(v.dot(b[i])));
    }
    res=res1+res2;
    double st=std::abs(v.dot(dc));
    return st>res;
}

void BoundingBody::setColor(Vector3d color_p)
{
    this->color=color_p;
}
void BoundingBody::setBoxColor(Vector3d color_p)
{
    this->box_color=color_p;
}

void BoundingBody::setCollisionColor(Vector3d color_p)
{
    this->collision_color=color_p;
}
Vector3d BoundingBody::getCenter()
{
    return this->center;
}

Vector3d BoundingBody::getColor(){
    return this->color;
}

vec3dd BoundingBody::getP(){
    return this->p;
}

vecintd BoundingBody::getInd(){
    return this->ind;
}

Vector3d BoundingBody::getBodyCenter()
{
    return this->bodycenter;
}

void BoundingBody::setRotation(Quat4d q)
{
    this->q_now=q;
    this->q_now.normalize();
    //this->box_calculated=false;
}
void BoundingBody::setZoom(double zoom_p)
{
    this->min-=center;
    this->max-=center;
    this->min/=zoom_val;
    this->max/=zoom_val;
    this->zoom_val=zoom_p;
    this->min*=zoom_val;
    this->max*=zoom_val;
    this->min+=center;
    this->max+=center;
}
void BoundingBody::setCenter(Vector3d center_p)
{
    this->min-=center;
    this->max-=center;
    this->center=center_p;
    this->min+=center;
    this->max+=center;
}

void BoundingBody::addCollision(BoundingBody &b)
{
    this->collision.push_back(&b);
}

void BoundingBody::clearCollision()
{
    this->collision.clear();
}


void BoundingBody::move(Vector3d motion)
{
    this->center+=motion;
    this->min+=motion;
    this->max+=motion;
    this->bodycenter+=motion;
    //this->box_calculated=false;
}
void BoundingBody::zoom(double z_p)
{
    this->min-=center;
    this->max-=center;
    this->zoom_val*=z_p;
    this->min*=z_p;
    this->max*=z_p;
    this->min+=center;
    this->max+=center;
    //this->box_calculated=false;
}
void BoundingBody::rotate(Quat4d rotation)
{
    this->q_now=rotation*q_now;
    this->q_now.normalize();
    //this->box_calculated=false;
}
