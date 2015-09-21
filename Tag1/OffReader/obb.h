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

class OBB{
    public:
        //methodes
        OBB();
        OBB(const vecvec3d *p, const vecvecuint ind, Vector3d color_p);

        //OBB( OBB &o);
        void draw();
        void caluculateC(const vecvec3d *p, const vecvecuint ind);
        void caluculateHalfL();
        void setCorner(const vecvec3d *p, const vecvecuint ind);
        bool intersect(OBB &B);
        void splitOBB(const OBB& A, OBB& A1, OBB& A2);
        static bool sortFkt0(const Vector3d &a,const Vector3d &b);
        static bool sortFkt1(const Vector3d &a,const Vector3d &b);
        static bool sortFkt2(const Vector3d &a,const Vector3d &b);
        bool intersectAxis(Vector3d &v, vecvec3d &a, vecvec3d &b, Vector3d &alpha, Vector3d &beta, Vector3d &dc);
        void setBodyCenter(Vector3d center_b);
        Matrix4d dyadicProdukt(Vector3d v1,Vector3d v2);

        //variablen
        std::vector<Vector3d> points;
        std::vector<Vector3d> corner;
        std::vector<Vector3d> axis;
        Vector3d color;
        //Vector3d center;
        Vector3d halflength;
        Vector3d normal;
        Vector3d base;
        Vector4d eigenvaluesC;
        Matrix4d R;
        Matrix4d C;
        double min_x,min_y,min_z,max_x,max_y,max_z;

public:
    void setBoxColor(Vector3d color_p);
    void setCollisionColor(Vector3d color_p);
    Vector3d getCenter();
    Vector3d getBodyCenter();
    Vector3d getColor();
    void setRotation(Quat4d q);
    void setCenter(Vector3d center_p);
    void move(Vector3d motion);
    void rotate(Quat4d rotation);

    Quat4d q_now;

protected:
    Vector3d min, max, center, bodycenter;
    bool selected;
    Vector3d box_color,collision_color,selected_color,selected_collision_color;
};


#endif // OBB_H

