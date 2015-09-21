#ifndef BVT_H
#define BVT_H

#include "vecmath.h"
#include <vector>
#include <map>
#include "Sphere.h"
#include <iostream>
#include <algorithm>
#include "obb.h"

typedef unsigned int uint;

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
        const vecvec3d *points;

		/// mass center of point set
        Vector3d mass_center;
        Vector3d center;
        Quat4d rot;

		/// inetria matrix of point set
        Matrix4d inertia;
	
		/// smallest enclosing sphere of point set	
        OBB box;
	
        BVT * left;
        BVT * right;

        void init(bool init_midtriange);
	public:

		/// create new node
        BVT (const vecvecuint &idx_p, const vecvec3d *points_p, uint depth);
        BVT (const vecvec3d &triMids_p, const vecvecuint &idx_p, const vecvec3d *points_p, vecvec3d *ball_points, uint depth);

        void draw();
        bool drawBoxes;
        bool drawModel;
        bool intersection;
        uint drawDepth;
        Vector4d model_color;

        uint actualDepth;

		/// get children
        BVT * getLeft() {return left;}
        BVT * getRight() {return right;}
        vecvec3d getPoints(){return *points;}
        vecvecuint getIdx(){return idx;}

		/// one recursion step to create left and right child
		void split ();
        Vector3d splitAxis;


        static bool sortFkt0(const Vector3d &a, const Vector3d &b);
        static bool sortFkt1(const Vector3d &a, const Vector3d &b);
        static bool sortFkt2(const Vector3d &a, const Vector3d &b);

        void drawPoints(Vector3d color=Vector3d(0.8,0.3,0.2));

		/// get sphere
        OBB& getBox() {return box;}

		/// anzahl der Punkte
        int nr_of_points () {return points->size();}

        bool intersect(OBB &S);
        bool intersect(BVT &S);

        int createTree(const uint minPoints);
};






#endif //BVT
