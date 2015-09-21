#ifndef SPHERE_H
#define SPHERE_H

#include <vector>

#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

#include "vecmath.h"

class Sphere {
  public:

  Vector3d center;
  double radius;

  Sphere();

  // Konstruiere die Umkugel fuer einen Punkt
  Sphere(Vector3d& a);
  
  // Konstruiere die Umkugel fuer zwei Punkte
  Sphere(Vector3d& a, Vector3d& b);
  
  // Konstruiere die Umkugel fuer drei Punkte
  Sphere(Vector3d& a, Vector3d& b, Vector3d& c);

  // Konstruiere die Umkugel fuer vier Punkte
  Sphere(Vector3d& a, Vector3d& b, Vector3d& c, Vector3d& d);

  // Berechne die kleinste einschliessende Kugel fuer die Punktmenge p
  Sphere(const std::vector<Vector3d>& p);

  // Berechne den Schwerpunkt der Punktmenge
  Sphere com(const std::vector<Vector3d>& p);

	/// Malt die Kugel
	void draw(Vector3d color = Vector3d(0.7,0.6,0.7)) const;
    bool intersect(Sphere const &S) const;
    bool intersect(Vector3d const &p) const;

  private:

  // Berechne die kleinste einschliessende Kugel fuer n Punkte, wobei
  // die Punkte q1,q2 und q3 auf dem Rand der gesuchten Kugel liegen
  Sphere ses3(int n,std::vector<Vector3d>& p,Vector3d& q1,Vector3d& q2,Vector3d& q3);
  
  // Berechne die kleinste einschliessende Kugel fuer n Punkte,
  // wobei die Punkte q1 und q2 auf dem Rand der gesuchten Kugel liegen
  Sphere ses2(int n,std::vector<Vector3d>& p,Vector3d& q1,Vector3d& q2);
  
  // Berechne die kleinste einschliessende Kugel fuer n Punkte,
  // wobei der Punkt q auf dem Rand der gesuchten Kugel liegt
  Sphere ses1(int n,std::vector<Vector3d>& p,Vector3d& q1);

};

#endif
