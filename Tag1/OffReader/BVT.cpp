#include "BVT.h"



#define maximal_points 10

#define uint unsigned int
using namespace std;

// Construktor
BVT::BVT (const vecvecuint& idx_p, const vecvec3d& points_p,uint depth):
    triMids(idx_p.size()), idx(idx_p), points(points_p), ball(points_p), actualDepth(depth)
{
    init(true);
}

BVT::BVT(const vecvec3d &triMids_p, const vecvecuint &idx_p, const vecvec3d &points_p, unsigned int depth):
    triMids(triMids_p), idx(idx_p), points(points_p), ball(points_p), actualDepth(depth)
{
    init(false);
}

void BVT::init(bool init_midtriange)
{
    uint i,j;
    Vector3d r;
    for(i=0;i<idx.size();++i){
        r=0;
        for (j = 0; j < 3; ++j){
            r += points[idx[i][j]]/3;
        }
        if(init_midtriange)
            triMids[i]=r;
        mass_center+=r;
    }
    mass_center /= idx.size();

    // Compute inetria matrix

    for(unsigned int i=0;i<points.size ();i++) {
        r=-mass_center;
        for (j = 0; j < 3; ++j){
            r += points[idx[i][j]]/3;
        }

        inertia(0,0) += r[1]*r[1]+r[2]*r[2];
        inertia(1,1) += r[0]*r[0]+r[2]*r[2];
        inertia(2,2) += r[0]*r[0]+r[1]*r[1];
        inertia(0,1) -= r[0]*r[1];
        inertia(0,2) -= r[0]*r[2];
        inertia(1,2) -= r[1]*r[2];
    }

    /// Due to the fact that we work with Matrix4d, expand the last row/column by (0,0,0,1)
    inertia(0,3) = inertia(3,0) = 0.0;
    inertia(1,3) = inertia(3,1) = 0.0;
    inertia(2,3) = inertia(3,2) = 0.0;
    inertia(3,3) = 1.0;

    // Kinder! Blaetter haben NULL-Pointer als Kinder!
    left = NULL;
    right = NULL;
}


/********************************************************
** Construcs an optimal splitting of points into two disjoint sets
** links and rechts and increases the BVT-tree by 1. 
** The splitting follows the idea of the lecture:
** (1) Compute the inertia matrix M of points
** (2) Compute the eigenvalues/eigenvectors of M by M.jacoby() (vecmath!)
** (3) Let v be the most stable eigenvector and c be the mass center.
**     Split points by plane p: (x-c)^T*v = 0
*********************************************************/

void BVT::split ()
{
    vecvec3d left, right, allSorted=this->getPoints();
    std::map<vecvec3d,vecvecuint> midTriangMap;




	// Computes the eigenvalues/vectors from the inertia matrix
	Vector4d eigenvalue;
	Matrix4d eigenvector;
	int nrot; /// not important

    /// Dont forget to compute inertia first!
    inertia.jacobi (eigenvalue, eigenvector, nrot);
    Quat4d q;
    q.set(eigenvector.inverse(eigenvector));

    //search for eigenvector to the smallest eigenvalue
    //splitting along this vector

    int m=0;
    if(eigenvalue[m]>eigenvalue[1])
        m=1;
    if(eigenvalue[m]>eigenvalue[2])
        m=2;

    splitAxis=Vector3d(eigenvector(m,0),eigenvector(m,1),eigenvector(m,2));
    for(uint i=0;i<allSorted.size();i++)
        allSorted[i]=q*allSorted[i];
    switch(m){
        case 0:std::sort(allSorted.begin(),allSorted.end(),&sortFkt0);break;
        case 1:std::sort(allSorted.begin(),allSorted.end(),&sortFkt1);break;
        case 2:std::sort(allSorted.begin(),allSorted.end(),&sortFkt2);break;
    }
    q.set(eigenvector);
    for(uint i=0;i<allSorted.size();i++)
        allSorted[i]=q*allSorted[i];
        right.resize(allSorted.size()/2);
    if(allSorted.size()%2==0)
        left.resize(allSorted.size()/2);
    else
        left.resize(allSorted.size()/2+1);

    for(uint i=0;i<allSorted.size()/2;i++){
        right[i]=allSorted[i];
        left[i]=allSorted[allSorted.size()/2+i];
    }
    uint t=left.size()-1;
    if(allSorted.size()%2==1)
        left[t]=allSorted[allSorted.size()-1];




	/// Kinder sind wieder Baeume! Wichtig das NEW!
    //left = new BVT (left,this->actualDeep+1);
    //right = new BVT (right,this->actualDeep+1);

}

bool BVT::sortFkt0(const Vector3d &a, const Vector3d &b){
    return a[0]>b[0];
}
bool BVT::sortFkt1(const Vector3d &a, const Vector3d &b){
    return a[1]>b[1];
}
bool BVT::sortFkt2(const Vector3d &a, const Vector3d &b){
    return a[2]>b[2];
}

void BVT::drawPoints(Vector3d color)
{
    glColor3d( color[0], color[1], color[2]);


    glPointSize(5*2.5);
    glBegin(GL_POINTS);
    for (uint i = 0; i < this->points.size(); ++i) {
        glVertex3dv(this->points[i].ptr());
    }
    glEnd();
}

bool BVT::intersect(const Sphere &S)
{
    if (S.intersect(this->ball)){
        if(NULL!=this->left){
            if (this->left->intersect(S))
                return true;
        }
        if(NULL!=this->right){
            if (this->right->intersect(S))
                return true;
        } else if(NULL==this->left){
            for (uint i = 0; i < this->points.size(); ++i) {
                if (S.intersect(this->points[i]))
                    return true;
            }
        }

    }
    return false;
}

bool BVT::intersect(const BVT &S)
{
    if (S.ball.intersect(this->ball)){
        if(NULL!=S.left){
            if (S.left->intersect(*this))
                return true;
        }
        if(NULL!=S.right){
            if (S.right->intersect(*this))
                return true;
        } else if(NULL==S.left){
            if(NULL!=this->left){
                if (this->left->intersect(S))
                    return true;
            }
            if(NULL!=this->right){
                if (this->right->intersect(S))
                    return true;
            } else if(NULL==this->left){
                for (uint i = 0; i < this->points.size(); ++i) {
                    if (S.ball.intersect(this->points[i]))
                        return true;
                }
            }
        }

    }
    return false;
}
