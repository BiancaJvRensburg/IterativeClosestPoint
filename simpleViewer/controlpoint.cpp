#include "controlpoint.h"
#include <math.h>

#include <QGLViewer/manipulatedFrame.h>

ControlPoint::ControlPoint()
{
    this->p = Vec(0,0,0);
    initialise();
}

ControlPoint::ControlPoint(const Vec& p)
{
    this->p = p;
    initialise();
}

void ControlPoint::initialise(){
    mf = ManipulatedFrame();
    mf.setPosition(this->p.x, this->p.y, this->p.z);
    connect(&mf, &ManipulatedFrame::manipulated, this, &ControlPoint::cntrlMoved);
}

void ControlPoint::draw(){
    glPushMatrix();
    glMultMatrixd(mf.matrix());

    if(mf.grabsMouse()) glColor3f(0, 1, 1);
    else glColor3f(0.6f, 0, 0.4f);

    glPointSize(15.0);
    glBegin(GL_POINTS);
        glVertex3d(0, 0, 0);
    glEnd();

    glPointSize(1.0);
    glColor3f(1.0,1.0,1.0);

    glPopMatrix();
}

void ControlPoint::cntrlMoved(){
    double x,y,z;
    mf.getPosition(x,y,z);
    p.x = x;
    p.y = y;
    p.z = z;

    Q_EMIT ControlPoint::cntrlPointTranslated();
    //std::cout << p.x << " , " << p.y << " , " << p.z << std::endl;
}

void ControlPoint::scale(float &s){
    p *= s;
    mf.setPosition(p);
}

void ControlPoint::translate(Vec &t){
    p += t;
    mf.setPosition(p);
}

void ControlPoint::move(Vec &t){
    p = t;
    mf.setPosition(p);
}
