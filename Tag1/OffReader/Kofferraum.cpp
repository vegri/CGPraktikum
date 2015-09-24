#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QGLWidget>
#include <QKeyEvent>
#include <QMessageBox>
#include <QHBoxLayout>

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>

#include "Kofferraum.h"

CGMainWindow::CGMainWindow (QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow (parent, flags) {
    resize (604, 614);


    // Create a menu
    QMenu *file = new QMenu("&File",this);
    file->addAction ("Load polyhedron", this, SLOT(loadPolyhedron()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 340-30-460", this, SLOT(loadPackage1()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 410-160-1490", this, SLOT(loadPackage2()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 340-40-740", this, SLOT(loadPackage3()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 140-190-800", this, SLOT(loadPackage4()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 110-50-480", this, SLOT(loadPackage5()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 360-40-600", this, SLOT(loadPackage6()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 250-100-310", this, SLOT(loadPackage7()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 310-190-320", this, SLOT(loadPackage8()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 470-440-680", this, SLOT(loadPackage9()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load package 310-280-450", this, SLOT(loadPackage10()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load all packages", this, SLOT(loadAllPackages()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);
    menuBar()->addMenu(file);

    // Create a nice frame to put around the OpenGL widget
    QFrame* f = new QFrame (this);
    f->setFrameStyle(QFrame::Sunken | QFrame::Panel);
    f->setLineWidth(2);


    // Create our OpenGL widget
    ogl = new CGView (this,f);
    // Put the GL widget inside the frame
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(ogl);
    layout->setMargin(0);
    f->setLayout(layout);

    setCentralWidget(f);
    ogl->zoom=0.002;
    statusBar()->showMessage("Ready",1000);
    loadPackage1();
    ogl->packageList[0].setCenter(Vector3d(0,0,255));
    //ogl->packageList[0].setCenter(Vector3d(-584,482.873,218.602));
    //ogl->packageList[0].setRot(Quat4d(-0.434506,0.206074,-0.465432,0.743043));
    loadPoly("../TestKofferraumIKEA.off");
    //loadPoly("../../num_15_tris.off");

    //loadPackage2();
}


void CGMainWindow::loadPackage1(){
    ogl->packageList.push_back(Package( 50, 50, 50));
    ogl->updateGL();
}
void CGMainWindow::loadPackage2(){
    ogl->packageList.push_back(Package(410,160,1490));
    ogl->updateGL();
}
void CGMainWindow::loadPackage3(){
    ogl->packageList.push_back(Package(340, 40,740));
    ogl->updateGL();
}
void CGMainWindow::loadPackage4(){
    ogl->packageList.push_back(Package(140,190,800));
    ogl->updateGL();
}
void CGMainWindow::loadPackage5(){
    ogl->packageList.push_back(Package(110, 50,480));
    ogl->updateGL();
}
void CGMainWindow::loadPackage6(){
    ogl->packageList.push_back(Package(360, 40,600));
    ogl->updateGL();
}
void CGMainWindow::loadPackage7(){
    ogl->packageList.push_back(Package(250,100,310));
    ogl->updateGL();
}
void CGMainWindow::loadPackage8(){
    ogl->packageList.push_back(Package(310,190,320));
    ogl->updateGL();
}
void CGMainWindow::loadPackage9(){
    ogl->packageList.push_back(Package(470,440,680));
    ogl->updateGL();
}
void CGMainWindow::loadPackage10(){
    ogl->packageList.push_back(Package(310,280,450));
    ogl->updateGL();
}
void CGMainWindow::loadAllPackages(){

    for(uint i=0;i<1;++i){
        ogl->packageList.push_back(Package( 50, 50, 50));
        ogl->packageList.push_back(Package(410,160,1490));
        ogl->packageList.push_back(Package(340, 40,740));
        ogl->packageList.push_back(Package(140,190,800));
        ogl->packageList.push_back(Package(110, 50,480));
        ogl->packageList.push_back(Package(360, 40,600));
        ogl->packageList.push_back(Package(250,100,310));
        ogl->packageList.push_back(Package(310,190,320));
        ogl->packageList.push_back(Package(470,440,680));
        ogl->packageList.push_back(Package(310,280,450));
    }

    srand(time(NULL));

    Vector3d zero=0;
    if(ogl->bootList.size()>0)
        zero+=ogl->bootList[ogl->bootList.size()-1]->getCenter();

    for (uint i = 0; i < ogl->packageList.size(); ++i) {
        Vector3d epsilon=Vector3d(rand()*ULONG_MAX,rand()*ULONG_MAX,rand()*ULONG_MAX).normalized()*2-1;
        Quat4d rot=Quat4d(rand(),rand(),rand(),rand());
        rot.normalize();
        ogl->packageList[i].setCenter(zero+epsilon);
        ogl->packageList[i].rotate(rot);
    }
    ogl->updateGL();
}

CGMainWindow::~CGMainWindow () {}

void CGMainWindow::loadPolyhedron() {
    QString filename = QFileDialog::getOpenFileName(this, "Load polyhedron ...", QString(), "OFF files (*.off)" );

    if (filename.isEmpty()) return;
    statusBar()->showMessage ("Loading polyhedron ...");
    loadPoly(filename);
}

void CGMainWindow::loadPoly(QString filename){


    std::ifstream file(filename.toLatin1());
    int vn,fn,en;

    Vector3d min_v = +std::numeric_limits<double>::max();
    Vector3d max_v = -std::numeric_limits<double>::max();

    std::string s;
    file >> s;

    file >> vn >> fn >> en;
    std::cout << "number of vertices : " << vn << std::endl;
    std::cout << "number of faces    : " << fn << std::endl;
    std::cout << "number of edges    : " << en << std::endl;

    vecvec3d *coords=new vecvec3d(vn);

    for(int i=0;i<vn;i++) {
        file >> (*coords)[i][0] >> (*coords)[i][1] >> (*coords)[i][2];
        (*coords)[i]=(*coords)[i];
        for(int j=0;j<3;++j) {
            if ((*coords)[i][j] < min_v[j]) min_v[j] = (*coords)[i][j];
            if ((*coords)[i][j] > max_v[j]) max_v[j] = (*coords)[i][j];
        }

    }

    vecvecuint idxs(fn);

    for(int i=0;i<fn;i++) {
        int k;
        file >> k;
        idxs[i].resize(k);
        for(int j=0;j<k;j++) file >> idxs[i][j];
    }

    file.close();

    std::cout << "min = " << min_v << std::endl;
    std::cout << "max = " << max_v << std::endl;

    Vector3d extent = max_v - min_v;
    ogl->zoom = 1.5/extent.maxComp();

    Vector3d shift=(min_v+max_v)/2;
    for(int i=0;i<vn;i++)
        (*coords)[i]-=shift;

    ogl->center = 0;

    ogl->bootList.push_back(new BVT(idxs,coords,0));

    BVT * act=ogl->bootList.at(ogl->bootList.size()-1);
    act->drawModel=true;
    uint j=1;
    act->createTree(j);

    //DEBUG
    ogl->q_old=Quat4d(3.141/4,Vector3d(0,0,1))*Quat4d(3.141/2,Vector3d(1,0,0));
    ogl->q_now=ogl->q_old;
    //END DEBUG

    ogl->updateGL();
    statusBar()->showMessage ("Loading polyhedron done.",3000);
}

CGView::CGView (CGMainWindow *mainwindow,QWidget* parent ):
    QGLWidget (parent), mouse_mode(Qt::NoButton),SLERP(0),
    EULER_ANGLES(1){
    picked=-1;
    picked_active=false;
    projRot=false;
    depth=0;
    drawObb=false;
    setFocusPolicy(Qt::StrongFocus);
}

void CGView::initializeGL() {
    qglClearColor(Qt::white);
    //zoom = 1.0;
    center = 0.0;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

}

void CGView::paintModel() {
    glLineWidth(2.0);

    for(unsigned int i=0;i<ifs.size();++i) {
        glBegin(GL_LINE_LOOP);
        for(unsigned int j=0;j<ifs[i].size();++j)
            glVertex3dv(coord[ifs[i][j]].ptr());
        glEnd();
    }
}

void CGView::paintGL() {
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    Matrix4d R=Matrix4d(q_now);

    // draw the rotated model
    glLoadIdentity();

    glTranslated(0, 0, -3);
    glMultMatrixd(R.transpose().ptr());
    glScaled(zoom, zoom, zoom);
    glTranslated(-center[0],-center[1],-center[2]);

    if(drawObb)
        this->testObb.draw();

    for (uint j = 0; j < this->bootList.size(); ++j) {
        this->bootList[j]->resetCollision();
    }
    for (uint i = 0; i < this->packageList.size(); ++i) {
        this->packageList[i].resetCollision();
        for (uint j = 0; j < this->packageList.size(); ++j){
            if (i!=j && this->packageList[i].intersect(this->packageList[j])){
                this->packageList[i].setCollision(this->packageList[j]);
            }
        }
        for (uint j = 0; j < this->bootList.size(); ++j) {
            this->bootList[j]->intersect(this->packageList[i]);
        }
    }

    for (uint i = 0; i < this->packageList.size(); ++i) {
        this->packageList[i].draw();
    }
    for (uint i = 0; i < this->bootList.size(); ++i) {
        this->bootList[i]->draw(depth);
    }

    glColor4f(0.0, 0.3, 0.6, 0.7);
    paintModel();

    //DEBUG
    glColor4f(1.,0.,0.,1.);
    glLineWidth(2.);
    glBegin(GL_LINES);
    glVertex3dv(d_ray_d.ptr());
    glVertex3dv(d_ray_f.ptr());
    glEnd();
}

void CGView::resizeGL(int width, int height) {
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width > height) {
        double ratio = width/(double) height;
        gluPerspective(45,ratio,1.0,10000.0);
    } else {
        double ratio = height/(double) width;
        gluPerspective(45,1.0/ratio,0.01,10000.0);
    }
    currentWidth = width;
    currentHeight = height;
    glMatrixMode (GL_MODELVIEW);
}

void CGView::worldCoord(int x, int y, int z, Vector3d &v) {
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    GLdouble M[16], P[16];
    glGetDoublev(GL_PROJECTION_MATRIX,P);
    glGetDoublev(GL_MODELVIEW_MATRIX,M);
    gluUnProject(x,viewport[3]-1-y,z,M,P,viewport,&v[0],&v[1],&v[2]);
}

void CGView::mousePressEvent(QMouseEvent *event) {
    oldX = event->x();
    oldY = event->y();
    double epsilon=0.01/zoom;

    Vector3d dir, dir_n, near_l;
    worldCoord(oldX,oldY,1,dir);
    worldCoord(oldX,oldY,-1,near_l);

    //DEBUG
    d_ray_f=near_l;
    d_ray_d=dir;

    dir=dir-near_l;
    dir_n=dir.normalized();
    double min_z=1e300,act_z;
    int loc_picked=-1;
    Vector3d hit_point;

    switch (event->button()) {
    case Qt::LeftButton:{
        bool fixedMoveDir=false;

        for (uint i = 0; i < this->packageList.size(); ++i) {
            Vector3d loc_c=this->packageList[i].getCenter()-near_l;
            double dist=(loc_c-dir_n*(loc_c*dir_n)).length();
            if(dist<epsilon+this->packageList[i].getDiameter()/2){
                this->packageList[i].getDist(near_l,dir_n,epsilon,dist,act_z);
                if(dist<epsilon && min_z>act_z){
                    loc_picked=i;
                    min_z=act_z;
                    fixedMoveDir=true;
                } else if(this->packageList[i].getHit(near_l,dir_n,hit_point,act_z) &&
                        min_z>act_z){
                    loc_picked=i;
                    min_z=act_z;
                    fixedMoveDir=false;
                    hit=hit_point-this->packageList[i].getCenter();
                }
            }
        }

        if(picked<this->packageList.size()){
            this->packageList[picked].pick(false);
            this->packageList[picked].setMoveDir(false);
        }

        if(loc_picked!=-1 && ((uint) loc_picked) <this->packageList.size()){
            if(fixedMoveDir){
                this->packageList[loc_picked].setMoveDir(true);
                this->hit=Vector3d(0);
            }
            this->picked=loc_picked;
            this->packageList[picked].pick(true);
            picked_active=true;
        } else {
            picked_active=false;
        }
        this->mouse_mode=Qt::LeftButton;
    }
        break;
    case Qt::MidButton:
        this->mouse_mode=Qt::MidButton;
        break;
    case Qt::RightButton:{
        if(picked< this->packageList.size()){
            Vector3d loc_c=this->packageList[picked].getCenter()-near_l;
            double dist=(loc_c-dir_n*(loc_c*dir_n)).length();
            this->packageList[picked].getDistCircleLine(near_l, dir_n, epsilon, dist, act_z, hit_point);
            if(dist<epsilon){
                this->projRot=true;
            } else {
                this->projRot=false;
            }
            this->packageList[picked].markRot();
        }

        mouseToTrackball(oldX, oldY, currentWidth, currentHeight);
        this->trackballU=this->trackballV;
        this->mouse_mode=Qt::RightButton;
    }
        break;
    default:
        break;
    }
    updateGL();
    updateGL();
}

void CGView::mouseReleaseEvent(QMouseEvent *event) {
    switch ((uint) mouse_mode) {
    case Qt::LeftButton:
        if(this->picked<this->packageList.size())
            this->packageList[picked].setMoveDir(false);
    case Qt::MidButton:
        break;
    case Qt::RightButton:{
        mouseToTrackball(event->x(), event->y(), currentWidth, currentHeight);
        Quat4d q;
        trackball(trackballU, trackballV, q);
        if (q.lengthSquared() > 0) {
            if(this->picked_active){
                this->packageList[picked].rotateMarked(q);
            } else {
                q_now = q * q_old;
            }
        }
        if(!this->picked_active)
            q_old = q_now;
        else
            this->packageList[picked].removeRot();
        this->projRot=false;
        updateGL();
    }
        break;
    default:
        break;
    }
}

void CGView::wheelEvent(QWheelEvent* event) {
    if (event->delta() < 0) zoom *= 1.2; else zoom *= 1/1.2;
    update();
}

void CGView::mouseMoveEvent(QMouseEvent* event){
    Vector3d u;
    Vector3d v;

    switch ((uint) mouse_mode) {
    case Qt::LeftButton:{
        if(picked<this->packageList.size() && picked_active){
            int x=event->x(),y=event->y();
            worldCoord(x, y, 0, u);
            worldCoord(x, y, -42, v);

            Vector3d dp = u - v;
            dp.normalize();

            Vector3d pack_center=this->packageList[picked].getCenter()+hit;
            this->packageList[picked].move(u+dp*(dp*(pack_center-u))-pack_center);
        } else {
            Vector3d move_vec=Vector3d((1.0*event->x()-oldX)/currentWidth,-(1.0*event->y()-oldY)/currentHeight,0);
            this->center+=q_now.conjugate()*move_vec/zoom;
            oldX=event->x();
            oldY=event->y();
        }
    }
        break;
    case Qt::MidButton:
        break;
    case Qt::RightButton:{
        mouseToTrackball(event->x(), event->y(), currentWidth, currentHeight);
        Quat4d q;
        trackball(trackballU, trackballV, q);
        if (q.lengthSquared() > 0) {
            if(this->picked_active){
                this->packageList[picked].rotateMarked(q);
            } else {
                q_now = q * q_old;
            }
        }
    }
        break;
    default:
        break;
    }
    updateGL();
}

void CGView::mouseToTrackball(int x, int y, int w, int h){
    double r,cx,cy;

    if(picked_active && picked<this->packageList.size()){
        Vector3d n,dir,c=this->packageList[picked].getCenter();
        double rad=this->packageList[picked].getDiameter()/2;
        worldCoord(x,y,0,n);
        worldCoord(x,y,-42,dir);
        dir=dir-n;
        dir.normalize();

        Vector3d loc_c=c-n;

        Vector3d point=dir*(dir*loc_c);
        Vector3d v=point-loc_c;
        if(v.length()>rad)
            return;

        v=v+dir*sqrt(rad*rad-v.lengthSquared());

        if(projRot){
            Vector3d h=this->packageList[picked].getRotProjection();
            v=v-h*(h*v);
            v.normalize();
            v=v*rad;
        }

        if(this->packageList[picked].d_ray_points.size()==0)
            this->packageList[picked].d_ray_points.push_back(v);

        if(this->packageList[picked].d_ray_points.size()<2){
            this->packageList[picked].d_ray_points.push_back(v);
        } else {
            this->packageList[picked].d_ray_points[1]=v;
        }

        trackballV=v;
        return;

        r = fmin(w, h) / 2.0;
        cx = w / 2.0;
        cy = h / 2.0;
    } else {
        r = fmin(w, h) / 2.0;
        cx = w / 2.0;
        cy = h / 2.0;
    }
    double dx = (x - cx) / r;
    double dy = -(y - cy) / r;
    double rho = hypot(dx, dy);
    double z = 0;
    if (rho > 1) {
        // pull (dx, dy) to the trackball, leave z=0
        dx /= rho;
        dy /= rho;
    } else {
        z = sqrt(1 - rho * rho);
    }
    trackballV[0] = dx;
    trackballV[1] = dy;
    trackballV[2] = z;
}

void CGView::trackball(Vector3d u, Vector3d v, Quat4d &q) {
    q.makeRotate(u,v);
    q.normalize();
}

void CGView::rot(GLdouble x, GLdouble y, GLdouble z){
    Quat4d t_q,q_x,q_y,q_z;
    q_x=Quat4d(std::sin(x/2),0,0,std::cos(x/2));
    q_y=Quat4d(0,std::sin(y/2),0,std::cos(y/2));
    q_z=Quat4d(0,0,std::sin(z/2),std::cos(z/2));

    t_q=q_x*q_z*q_y;
    if(this->picked<this->packageList.size() && this->picked_active==true){
        this->packageList[picked].rotate(t_q);
    } else {
        q_now=t_q*q_now;
        q_now.normalize();
        q_old=q_now;
    }
    updateGL();
}

void CGView::move(GLdouble x, GLdouble y, GLdouble z){
    if(this->picked<this->packageList.size() && this->picked_active==true){
        this->packageList[picked].move(Vector3d(x,y,z)/zoom);
    } else {
        this->center+=q_now*(Vector3d(x,y,z))/zoom;
    }
    updateGL();
}

void CGView::keyPressEvent(QKeyEvent *e) {
    //std::cout << e->key() << std::endl;
    switch (e->key()) {
    case Qt::Key_S: rot( 0.05,0,0); break;
    case Qt::Key_W: rot(-0.05,0,0); break;
    case Qt::Key_E: rot(0,-0.05,0); break;
    case Qt::Key_Q: rot(0, 0.05,0); break;
    case Qt::Key_A: rot(0,0,-0.05); break;
    case Qt::Key_D: rot(0,0, 0.05); break;
    case Qt::Key_Left:       move(-0.05,0,0); break;
    case Qt::Key_Right:      move( 0.05,0,0); break;
    case Qt::Key_Up:         move(0, 0.05,0); break;
    case Qt::Key_Down:       move(0,-0.05,0); break;
    case Qt::Key_NumberSign: move(0,0, 0.05); break;
    case Qt::Key_Plus:       move(0,0,-0.05); break;
    case Qt::Key_X:
        if(picked<this->packageList.size())
            this->packageList[this->picked].pick(false);
        this->picked=(this->packageList.size()+this->picked+1)%this->packageList.size();
        if(picked<this->packageList.size())
            this->packageList[this->picked].pick(true);
        break;
    case Qt::Key_Y:
        if(picked<this->packageList.size())
            this->packageList[this->picked].pick(false);
        this->picked=(this->packageList.size()+this->picked-1)%this->packageList.size();
        if(picked<this->packageList.size())
            this->packageList[this->picked].pick(true);
        break;
    case Qt::Key_N:
        ++depth;
        break;
    case Qt::Key_M:
        --depth;
        break;
    case Qt::Key_F:
        std::cout << this->q_now[0] << " " << this->q_now[1] << " "
                  << this->q_now[2] << " " << this->q_now[3] << "(q_now)" << std::endl;
        std::cout << this->q_old[0] << " " << this->q_old[1] << " "
                  << this->q_old[2] << " " << this->q_old[3] << "(q_old)" << std::endl;
        std::cout << this->center[0] << " " << this->center[1] << " "
                  << this->center[2] << "(center)" << std::endl;
        std::cout << this->zoom << "(zoom)" << std::endl;
        std::cout << this->picked << "(picked)" << std::endl;
        break;
    case Qt::Key_G:
        std::cout << this->packageList[picked].getRot()[0] << "," << this->packageList[picked].getRot()[1] << ","
                  << this->packageList[picked].getRot()[2] << "," << this->packageList[picked].getRot()[3] << "(rot)" << std::endl;
        std::cout << this->packageList[picked].getCenter()[0] << "," << this->packageList[picked].getCenter()[1] << ","
                  << this->packageList[picked].getCenter()[2] << "(center)" << std::endl;
        break;
    case Qt::Key_R:{
        uint n=this->packageList.size();
        vecvec3d *points=new vecvec3d(8*n);
        vecvecuint idx(n*3);
        for (uint i = 0; i < n; ++i) {
            vecvec3d tmp=this->packageList[i].getCorners();
            for (uint j = 0; j < 8; ++j) {
                (*points)[8*i+j]=tmp[j];
            }
            for (uint j = 0; j < 9; ++j) {
                if(j%3==0)
                    idx[i*3+j/3]=vecuint(3);
                idx[i*3+j/3][j%3]=8*i+j%8;
            }
        }
        testObb=AABB(points,idx,Vector3d(0.5,0.5,0));
        drawObb=true;
        break;
    }
    case Qt::Key_Space:{
        uint collResolvedPB=true; //Package-Boot
        uint collResolvedPP=true; //Package-Package
        srand(time(NULL));
        uint k=0;
        bool jumpRotation=false;
        while(collResolvedPB+collResolvedPP>0){//(collResolvedPB+collResolvedPP>1) && k<50
            collResolvedPB=0;
            collResolvedPP=0;
            uint n=0;

            if(k%30==0) jumpRotation=true;

            k++;
            //collision boot/packages
            for (uint i = 0; i < this->packageList.size(); ++i) {
                for (uint j = 0; j < this->bootList.size(); ++j) {
                    collResolvedPB+=resolveCollision(this->packageList[i],(*this->bootList[j]),jumpRotation);
                }
            }

            //collision packages/packages random
            for (uint i = 0; i < this->packageList.size(); ++i) {
                n=((uint) rand())%this->packageList.size();
                for (uint j = 0; j < this->packageList.size(); ++j){
                    if(j!=n)
                        this->packageList[n].resolveCollision(this->packageList[j]);
                }
            }

            //collision packages/packages deterministic
            for (uint i = 0; i < this->packageList.size(); ++i) {
                for (uint j = i+1; j < this->packageList.size(); ++j){
                    collResolvedPP+=this->packageList[i].resolveCollision(this->packageList[j]);
                }
            }
            std::cout << collResolvedPB <<std::endl;
            std::cout << collResolvedPP <<std::endl;
            jumpRotation=false;
            updateGL();
            updateGL();
        }

    }
        break;
    default:
        break;
    }
    updateGL();
    updateGL();
}
int CGView::resolveCollision(Package &box, BVT &off, bool jumpRot){
    this->collDir=Vector3d(1,0,0)*1e300;

    bool coll_occ=off.intersect(box);;
    uint j=0;
    vecvec3d potDir,triMids;
    Vector3d move_dir=0,rotDir=0;

    move_dir=box.packageInBox(off.getBox());
    if(move_dir.length()!=0)
        coll_occ=true;

    while(coll_occ && j<5){
        ++j;
        off.getIntersectDirs(potDir,triMids);
        move_dir=0;
        for (uint i = 0; i < potDir.size(); ++i) {
            move_dir+=potDir[i];
            rotDir+=((box.getCenter()-triMids[i])%potDir[i]);
        }

        move_dir=move_dir/potDir.size()*0.9;
        rotDir=rotDir/potDir.size();
        Quat4d rot=Quat4d(0.01,rotDir);
        if(jumpRot)rot=Quat4d(0.7,rotDir);
        if(move_dir.length()<box.getDiameter()+off.getBox().getDiameter()){
            if(move_dir.lengthSquared()<1e-10)
                move_dir=move_dir.normalized()*std::abs(box.getCenter().minComp())*1e-6;
            box.move(move_dir*0.01);
            //covers case rot=nan:
            if(rot.length()==rot.length())
            box.rotate(rot);
        }
        move_dir=box.packageInBox(off.getBox());
//        if(move_dir.length()!=0)
//            coll_occ=true;
        box.move(move_dir*0.01);

        off.resetCollision();
        coll_occ=off.intersect(box);
    }
    return potDir.size();
}

int main (int argc, char **argv) {
    QApplication app(argc, argv);

    if (!QGLFormat::hasOpenGL()) {
        qWarning ("This system has no OpenGL support. Exiting.");
        return 1;
    }

    CGMainWindow *w = new CGMainWindow(NULL);

    w->show();

    return app.exec();
}


