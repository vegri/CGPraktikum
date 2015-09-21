#ifndef BVT_H
#define BVT_H

#include "vecmath.h"
#include <vector>
#include "Sphere.h"


/// Klassische rekursive Datenstruktur!
/** In den Knoten speichern wir die Info (points_, und ball)
		Die Kinder muessen Pointer sein (c++!)
*/
class BVT
{

	private:

		const std::vector<Vector3d> points_;

		/// mass center of point set
		Vector3d mass_center_;

		/// inetria matrix of point set
		Matrix4d inertia_;
	
		/// smallest enclosing sphere of point set	
		const Sphere ball_;
	
		BVT * left_;
		BVT * right_;

	public:

		/// create new node
        BVT (const std::vector<Vector3d>& points, int depth);
        int actualDeep;

		/// get children
        BVT * left() {return left_;}
        BVT * right() {return right_;}
        void setBall(std::vector<Vector3d> points);
        std::vector<Vector3d> getPoints(){return points_;}
		/// one recursion step to create left and right child
		void split ();
        Vector3d splitAxis;


        static bool sortFkt0(const Vector3d &a, const Vector3d &b);
        static bool sortFkt1(const Vector3d &a, const Vector3d &b);
        static bool sortFkt2(const Vector3d &a, const Vector3d &b);

        void drawPoints(Vector3d color=Vector3d(0.8,0.3,0.2));

		/// get sphere
        const Sphere& ball() {return ball_;}

		/// anzahl der Punkte
        int nr_of_points () {return points_.size ();}

        bool intersect(const Sphere & S);
        bool intersect(const BVT & S);
};






#endif //BVT
