#ifndef OBB_H
#define OBB_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>

#if _MSC_VER
    #include <gl/glu.h>
#elif __APPLE__
  #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

#include "vecmath.h"
#include "boundingbody.h"

class OBB: public BoundingBody{

    public:
        //methodes
        OBB();
        OBB(const vec3dd &p_p, vecintd ind_p, Vector3d color_p);

        //OBB( OBB &o);
        void draw();
        void caluculateC();
        void caluculateHalfL();
        void setCorner();
        bool intersect(OBB &B);
        void splitOBB(const OBB& A, OBB& A1, OBB& A2);
        static bool sortFkt0(const Vector3d &a,const Vector3d &b);
        static bool sortFkt1(const Vector3d &a,const Vector3d &b);
        static bool sortFkt2(const Vector3d &a,const Vector3d &b);
        bool intersectAxis(Vector3d &v, vec3dd &a, vec3dd &b, Vector3d &alpha, Vector3d &beta, Vector3d &dc);
        void setBodyCenter(Vector3d center_b);
        Matrix4d dyadicProdukt(Vector3d v1,Vector3d v2);

        //variablen
        std::vector<Vector3d> points;
        std::vector<Vector3d> corner;
        std::vector<Vector3d> axis;
        Vector3d longestAxis;
        Vector3d color;
        //Vector3d center;
        Vector3d halflength;
        Vector3d normal;
        Vector3d base;
        Vector4d eigenvaluesC;
        Matrix4d R;
        Matrix4d C;
        double min_x,min_y,min_z,max_x,max_y,max_z;


};


#endif // OBB_H

