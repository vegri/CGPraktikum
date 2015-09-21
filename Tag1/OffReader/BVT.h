#ifndef BVT_H
#define BVT_H

#include "vecmath.h"
#include <vector>
#include <map>
#include "Sphere.h"
#include <iostream>
#include <algorithm>


typedef std::vector<Vector3d> vecvec3d;
typedef std::vector<uint> vecuint;
typedef std::vector<vecuint> vecvecuint;

/// Klassische rekursive Datenstruktur!
/** In den Knoten speichern wir die Info (points_, und ball)
		Die Kinder muessen Pointer sein (c++!)
*/
class BVT
{

	private:
        vecvec3d triMids;
        const vecvecuint idx;
        const vecvec3d points;

		/// mass center of point set
        Vector3d mass_center;

		/// inetria matrix of point set
        Matrix4d inertia;
	
		/// smallest enclosing sphere of point set	
        const Sphere ball;
	
        BVT * left;
        BVT * right;

        void init(bool init_midtriange);
	public:

		/// create new node
        BVT (const vecvecuint &idx_p, const vecvec3d &points_p, unsigned int depth);
        BVT (const vecvec3d &triMids_p, const vecvecuint &idx_p, const vecvec3d &points_p, unsigned int depth);

        int actualDepth;

		/// get children
        BVT * getLeft() {return left;}
        BVT * getRight() {return right;}
        void setBall(std::vector<Vector3d> points);
        std::vector<Vector3d> getPoints(){return points;}
		/// one recursion step to create left and right child
		void split ();
        Vector3d splitAxis;


        static bool sortFkt0(const Vector3d &a, const Vector3d &b);
        static bool sortFkt1(const Vector3d &a, const Vector3d &b);
        static bool sortFkt2(const Vector3d &a, const Vector3d &b);

        void drawPoints(Vector3d color=Vector3d(0.8,0.3,0.2));

		/// get sphere
        const Sphere& getBall() {return ball;}

		/// anzahl der Punkte
        int nr_of_points () {return points.size ();}

        bool intersect(const Sphere & S);
        bool intersect(const BVT & S);
};






#endif //BVT
