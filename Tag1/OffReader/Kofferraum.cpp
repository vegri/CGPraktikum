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
#include "OffReader.h"

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

    statusBar()->showMessage("Ready",1000);
    loadPackage1();
}


void CGMainWindow::loadPackage1(){
    ogl->packageList.push_back(Package(.340,.30,.460));
}
void CGMainWindow::loadPackage2(){
    ogl->packageList.push_back(Package(.410,.160,.1490));
}
void CGMainWindow::loadPackage3(){
    ogl->packageList.push_back(Package(.340,.40,.740));
}
void CGMainWindow::loadPackage4(){
    ogl->packageList.push_back(Package(.140,.190,.800));
}
void CGMainWindow::loadPackage5(){
    ogl->packageList.push_back(Package(.110,.50,.480));
}
void CGMainWindow::loadPackage6(){
    ogl->packageList.push_back(Package(.360,.40,.600));
}
void CGMainWindow::loadPackage7(){
    ogl->packageList.push_back(Package(.250,.100,.310));
}
void CGMainWindow::loadPackage8(){
    ogl->packageList.push_back(Package(.310,.190,.320));
}
void CGMainWindow::loadPackage9(){
    ogl->packageList.push_back(Package(.470,.440,.680));
}
void CGMainWindow::loadPackage10(){
    ogl->packageList.push_back(Package(.310,.280,.450));
}
void CGMainWindow::loadAllPackages(){

}

CGMainWindow::~CGMainWindow () {}

void CGMainWindow::loadPolyhedron() {
    QString filename = QFileDialog::getOpenFileName(this, "Load polyhedron ...", QString(), "OFF files (*.off)" );

    if (filename.isEmpty()) return;
    statusBar()->showMessage ("Loading polyhedron ...");
    std::ifstream file(filename.toLatin1());
    int vn,fn,en;

    ogl->min = +std::numeric_limits<double>::max();
    ogl->max = -std::numeric_limits<double>::max();

    std::string s;
    file >> s;

    file >> vn >> fn >> en;
    std::cout << "number of vertices : " << vn << std::endl;
    std::cout << "number of faces    : " << fn << std::endl;
    std::cout << "number of edges    : " << en << std::endl;

    ogl->coord.resize(vn);

    for(int i=0;i<vn;i++) {
        file >> ogl->coord[i][0] >> ogl->coord[i][1] >> ogl->coord[i][2];

        for(int j=0;j<3;++j) {
            if (ogl->coord[i][j] < ogl->min[j]) ogl->min[j] = ogl->coord[i][j];
            if (ogl->coord[i][j] > ogl->max[j]) ogl->max[j] = ogl->coord[i][j];
        }

    }

    ogl->ifs.resize(fn);

    for(int i=0;i<fn;i++) {
        int k;
        file >> k;
        ogl->ifs[i].resize(k);
        for(int j=0;j<k;j++) file >> ogl->ifs[i][j];
    }

    file.close();

    std::cout << "min = " << ogl->min << std::endl;
    std::cout << "max = " << ogl->max << std::endl;

    Vector3d extent = ogl->max - ogl->min;
    ogl->zoom = 1.5/extent.maxComp();

    ogl->center = (ogl->min+ogl->max)/2;

    ogl->updateGL();
    statusBar()->showMessage ("Loading polyhedron done.",3000);
}

CGView::CGView (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {

    move=false;
    picked=0;
    setFocusPolicy(Qt::StrongFocus);
}

void CGView::initializeGL() {
    qglClearColor(Qt::white);
    zoom = 1.0;
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

    // draw the rotated model
    glLoadIdentity();
    glTranslated(0, 0, -3);
    glMultMatrixd(currentRotationMatrix.transpose().ptr());
    glScaled(zoom, zoom, zoom);
    glTranslated(-center[0],-center[1],-center[2]);

    for (uint i = 0; i < this->packageList.size(); ++i) {
        this->packageList[i].draw();
    }

    glColor4f(0.0, 0.3, 0.6, 0.7);
    paintModel();

    //DEBUG
    glColor4f(1.,0.,0.,1.);
    glLineWidth(3.);
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
    double epsilon=0.01;


    if(move){
    } else {

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

        for (uint i = 0; i < this->packageList.size(); ++i) {
            Vector3d loc_c=this->packageList[i].getCenter()-near_l;
            double dist=(loc_c-dir_n*(loc_c*dir_n)).length();
            if(dist<epsilon+this->packageList[i].getDiameter()/2){
                this->packageList[i].getDist(near_l,dir_n,dist,act_z);
                if(dist<epsilon && min_z>act_z){
                    loc_picked=i;
                    min_z=act_z;
                }

            }
        }

        if(loc_picked>-1 && loc_picked<this->packageList.size())
            this->picked=loc_picked;


        q_old = q_now;
        if (animationRunning) {
            q_now = q_animated;
            q_old = q_animated;
            animationRunning = false;
        }
    }

}

void CGView::mouseReleaseEvent(QMouseEvent *event) {
    Vector3d u;
    Vector3d v;
    mouseToTrackball(oldX, oldY, currentWidth, currentHeight, u);
    mouseToTrackball(event->x(), event->y(), currentWidth, currentHeight, v);
    Quat4d q;
    trackball(u, v, q);
    if (q.lengthSquared() > 0) {
        q_now = q * q_old;
    }
    q_old = q_now;
    currentRotationMatrix.makeRotate(q_now);
    updateGL();
}

void CGView::wheelEvent(QWheelEvent* event) {
    if (event->delta() < 0) zoom *= 1.2; else zoom *= 1/1.2;
    update();
}

void CGView::mouseMoveEvent(QMouseEvent* event) {
    Vector3d u;
    Vector3d v;
    if(move){
        Vector3d move_vec=Vector3d((1.0*event->x()-oldX)/currentWidth,-(1.0*event->y()-oldY)/currentHeight,0);
        move_vec=q_now.conjugate()*move_vec/zoom;
        this->packageList[picked].move(move_vec);
        oldX=event->x();
        oldY=event->y();
    } else {

        mouseToTrackball(oldX, oldY, currentWidth, currentHeight, u);
        mouseToTrackball(event->x(), event->y(), currentWidth, currentHeight, v);
        Quat4d q;
        trackball(u, v, q);
        if (q.lengthSquared() > 0) {
            // sometimes mouseMove events occur even when the mouse stays on
            // the same pixel. The length of `q` becomes 0, and everything breaks
            // down. To prevent this, we check that `q` is not 0.
            q_now = q * q_old;
        }
        currentRotationMatrix.makeRotate(q_now);
    }
    updateGL();
}

void CGView::mouseToTrackball(int x, int y, int w, int h, Vector3d &v) {
    double r = fmin(w, h) / 2.0;
    double cx = w / 2.0;
    double cy = h / 2.0;
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
    v.v[0] = dx;
    v.v[1] = dy;
    v.v[2] = z;
}

void CGView::trackball(Vector3d u, Vector3d v, Quat4d &q) {
    q.set(1 + u.dot(v), u % v);
    q.normalize();
}

void CGView::keyPressEvent(QKeyEvent *e) {
    switch (e->key()) {
    case Qt::Key_Space:
        move=!move;
        std::cout << move << std::endl;
        break;
    case Qt::Key_X:
        this->packageList[this->picked].resetColor();
        this->picked=(this->packageList.size()+this->picked+1)%this->packageList.size();
        this->packageList[this->picked].setColor(Vector4d(0,0,0,1));
        break;
    case Qt::Key_Y:
        this->packageList[this->picked].resetColor();
        this->picked=(this->packageList.size()+this->picked-1)%this->packageList.size();
        this->packageList[this->picked].setColor(Vector4d(0,0,0,1));
        break;
    default:
        break;
    }
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


