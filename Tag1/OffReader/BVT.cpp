#include "BVT.h"



#define maximal_points 10

using namespace std;

// Construktor
BVT::BVT (const vecvecuint& idx_p, const vecvec3d *points_p, uint depth):
    triMids(idx_p.size()), idx(idx_p), points(points_p), ball(*points_p), actualDepth(depth)
{
    init(true);
}

BVT::BVT(const vecvec3d &triMids_p, const vecvecuint &idx_p, const vecvec3d *points_p, const vecvec3d ball_points, unsigned int depth):
    triMids(triMids_p), idx(idx_p), points(points_p), ball(ball_points), actualDepth(depth)
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
            r += (*points)[idx[i][j]]/3;
        }
        if(init_midtriange)
            triMids[i]=r;
        mass_center+=r;
    }
    mass_center /= idx.size();

    // Compute inetria matrix

    for(unsigned int i=0;i<idx.size ();i++) {
        r=-mass_center;
        for (j = 0; j < 3; ++j){
            r += (*points)[idx[i][j]]/3;
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
    vecvec3d left_vec, left_points, right_vec, right_points, allSorted=triMids;
    vecvecuint left_idx,right_idx;
    std::map<Vector3d,vecuint> midTriangMap;

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
    for(uint i=0;i<allSorted.size();i++){
        allSorted[i]=q*allSorted[i];
        midTriangMap[allSorted[i]]=idx[i];
    }
    switch(m){
        case 0:std::sort(allSorted.begin(),allSorted.end(),&sortFkt0);break;
        case 1:std::sort(allSorted.begin(),allSorted.end(),&sortFkt1);break;
        case 2:std::sort(allSorted.begin(),allSorted.end(),&sortFkt2);break;
    }
    q.set(eigenvector);
    //for(uint i=0;i<allSorted.size();i++)
    //    allSorted[i]=q*allSorted[i];

    right_vec.resize(allSorted.size()/2);
    if(allSorted.size()%2==0)
        left_vec.resize(allSorted.size()/2);
    else
        left_vec.resize(allSorted.size()/2+1);


    right_idx.resize(right_vec.size());
    left_idx.resize(left_vec.size());
    right_points.resize(right_vec.size()*3);
    left_points.resize(left_vec.size()*3);


    for(uint i=0;i<allSorted.size()/2;i++){
        right_vec[i]=q*allSorted[i];
        left_vec[i]=q*allSorted[allSorted.size()/2+i];
        right_idx[i]=midTriangMap[allSorted[i]];
        left_idx[i]=midTriangMap[allSorted[allSorted.size()/2+i]];
        for (uint j = 0; j < 3; ++j) {
            right_points[3*i+j]=(*points)[right_idx[i][j]];
            left_points[3*i+j]=(*points)[left_idx[i][j]];
        }
    }
    uint t=left_vec.size()-1;
    if(allSorted.size()%2==1){
        left_vec[t]=q*allSorted[allSorted.size()-1];
        left_idx[t]=midTriangMap[allSorted[allSorted.size()-1]];
        for (uint j = 0; j < 3; ++j) {
            left_points[3*t+j]=(*points)[left_idx[t][j]];
        }
    }

	/// Kinder sind wieder Baeume! Wichtig das NEW!
    left  = new BVT (left_vec,  left_idx,  points, left_points,  this->actualDepth+1);
    right = new BVT (right_vec, right_idx, points, right_points, this->actualDepth+1);

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
    for (uint i = 0; i < this->points->size(); ++i) {
        glVertex3dv((*points)[i].ptr());
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
            for (uint i = 0; i < this->points->size(); ++i) {
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
                for (uint i = 0; i < this->points->size(); ++i) {
                    if (S.ball.intersect(this->points[i]))
                        return true;
                }
            }
        }

    }
    return false;
}

int BVT::createTree(unsigned int &minPoints)
{
    if((this->getIdx()).size()<=minPoint) return;
    this->split();
    this->left->createTree(minPoint);
    this->right->createTree(minPoint);
}
