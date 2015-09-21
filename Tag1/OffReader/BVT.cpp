#include "BVT.h"

#include "Sphere.h"

#include <iostream>
#include <algorithm>

#define maximal_points 10

#define uint unsigned int
using namespace std;

// Construktor
BVT::BVT (const std::vector<Vector3d>& points,int depth) : points_ (points), ball_ (points)
{
  this->actualDeep=depth;
  for(unsigned int i=0;i<points_.size();i++)
    mass_center_ += points_[i];
  mass_center_ /= points_.size();

	// Compute inetria matrix
  for(unsigned int i=0;i<points_.size ();i++) {
    const Vector3d& r = points_[i] - mass_center_;
    inertia_(0,0) += r[1]*r[1]+r[2]*r[2];
    inertia_(1,1) += r[0]*r[0]+r[2]*r[2];
    inertia_(2,2) += r[0]*r[0]+r[1]*r[1];
    inertia_(0,1) -= r[0]*r[1];
    inertia_(0,2) -= r[0]*r[2];
    inertia_(1,2) -= r[1]*r[2];
  }
 
	/// Due to the fact that we work with Matrix4d, expand the last row/column by (0,0,0,1)
  inertia_(0,3) = inertia_(3,0) = 0.0;
  inertia_(1,3) = inertia_(3,1) = 0.0;
  inertia_(2,3) = inertia_(3,2) = 0.0;
  inertia_(3,3) = 1.0;
	
	// Kinder! Blaetter haben NULL-Pointer als Kinder!
	left_ = NULL;
	right_ = NULL;
}


/********************************************************
** Construcs an optimal splitting of points_ into two disjoint sets
** links and rechts and increases the BVT-tree by 1. 
** The splitting follows the idea of the lecture:
** (1) Compute the inertia matrix M of points_
** (2) Compute the eigenvalues/eigenvectors of M by M.jacoby() (vecmath!)
** (3) Let v be the most stable eigenvector and c be the mass center.
**     Split points_ by plane p: (x-c)^T*v = 0 
*********************************************************/

void BVT::split ()
{
    std::vector<Vector3d> left;
    std::vector<Vector3d> right;
    std::vector<Vector3d> allSorted=this->getPoints();



	// Computes the eigenvalues/vectors from the inertia matrix
	Vector4d eigenvalue;
	Matrix4d eigenvector;
	int nrot; /// not important

	/// Dont forget to compute inertia_ first!
	inertia_.jacobi (eigenvalue, eigenvector, nrot);
    Quat4d q;
    q.set(eigenvector.inverse(eigenvector));

	/// ADD YOUR CODE HERE!!!

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
    left_ = new BVT (left,this->actualDeep+1);
    right_ = new BVT (right,this->actualDeep+1);

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
    for (uint i = 0; i < this->points_.size(); ++i) {
        glVertex3dv(this->points_[i].ptr());
    }
    glEnd();
}

bool BVT::intersect(const Sphere &S)
{
    if (S.intersect(this->ball())){
        if(NULL!=this->left_){
            if (this->left_->intersect(S))
                return true;
        }
        if(NULL!=this->right_){
            if (this->right_->intersect(S))
                return true;
        } else if(NULL==this->left_){
            for (uint i = 0; i < this->points_.size(); ++i) {
                if (S.intersect(this->points_[i]))
                    return true;
            }
        }

    }
    return false;
}

bool BVT::intersect(const BVT &S)
{
    if (S.ball_.intersect(this->ball())){
        if(NULL!=S.left_){
            if (S.left_->intersect(*this))
                return true;
        }
        if(NULL!=S.right_){
            if (S.right_->intersect(*this))
                return true;
        } else if(NULL==S.left_){
            if(NULL!=this->left_){
                if (this->left_->intersect(S))
                    return true;
            }
            if(NULL!=this->right_){
                if (this->right_->intersect(S))
                    return true;
            } else if(NULL==this->left_){
                for (uint i = 0; i < this->points_.size(); ++i) {
                    if (S.ball_.intersect(this->points_[i]))
                        return true;
                }
            }
        }

    }
    return false;
}
