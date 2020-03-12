#include "curve.h"
#include "math.h"
#include <GL/gl.h>

Curve::Curve(unsigned int nbCP, std::vector<Vec>& cntrlPoints){
    this->nbU = 0;
    nbControlPoint = nbCP;

    for(unsigned int i=0; i<nbCP; i++){
        TabControlPoint.push_back(new ControlPoint(cntrlPoints[i]));
    }

    initConnections();
}

void Curve::initConnections(){
    for(unsigned int i=0; i<nbControlPoint; i++){
        connect(TabControlPoint[i], &ControlPoint::cntrlPointTranslated, this, &Curve::reintialiseCurve);
    }
}

void Curve::generateCatmull(unsigned int& n){
    unsigned int nbSeg = nbControlPoint-3;

    this->nbU = n - n%nbSeg;
    n = nbU;    // update the nbU in viewer
    this->knotIndex = 0;
    this->degree = 3;

    this->knotVector = generateCatmullKnotVector(0.3);
    catmullrom();
}

void Curve::reintialiseCurve(){
   catmullrom();
    Q_EMIT curveReinitialised();
}

std::vector<double> Curve::generateCatmullKnotVector(double alpha){
    std::vector<double> kv;
    kv.resize(static_cast<unsigned long long>(nbControlPoint));

    kv[0] = 0;

    for(unsigned int i=1; i<nbControlPoint; i++){
        Vec p = TabControlPoint[i]->getPoint() - TabControlPoint[i-1]->getPoint();
        kv[i] =  pow(p.norm(),alpha) + kv[i-1];
    }

    return kv;
}

// Catmull rom
void Curve::calculateCatmullPoints(Vec& c, double t){
    Vec p[4] = {TabControlPoint[knotIndex-1]->getPoint(), TabControlPoint[knotIndex]->getPoint(), TabControlPoint[knotIndex+1]->getPoint(), TabControlPoint[knotIndex+2]->getPoint()};

    double t0 = knotVector[knotIndex-1];
    double t1 = knotVector[knotIndex];
    double t2 = knotVector[knotIndex+1];
    double t3 = knotVector[knotIndex+2];

    Vec a1 = (t1-t)/(t1-t0)*p[0] + (t-t0)/(t1-t0)*p[1];
    Vec a2 = (t2-t)/(t2-t1)*p[1] + (t-t1)/(t2-t1)*p[2];
    Vec a3 = (t3-t)/(t3-t2)*p[2] + (t-t2)/(t3-t2)*p[3];

    Vec b1 = (t2-t)/(t2-t0)*a1 + (t-t0)/(t2-t0)*a2;
    Vec b2 = (t3-t)/(t3-t1)*a2 + (t-t1)/(t3-t1)*a3;

    c = (t2-t)/(t2-t1)*b1 + (t-t1)/(t2-t1)*b2;
}

void Curve::catmullrom(){
    unsigned int nbSeg = nbControlPoint-3;
    unsigned int uPerSeg = nbU/nbSeg;

    curve = new Vec[static_cast<unsigned long long>(nbU)];

    for(unsigned int j=1; j<=nbSeg; j++){
        unsigned int it=0;
        knotIndex = j;

        for(double i=knotVector[j]; i<knotVector[j+1]; i+=((knotVector[j+1]-knotVector[j])/static_cast<double>(uPerSeg))){
            if((j-1)*uPerSeg+it >= nbU) return;
            curve[(j-1)*uPerSeg+it] = Vec();
            calculateCatmullPoints(curve[(j-1)*uPerSeg+it], i);
            it++;
        }
    }
}

void Curve::draw(){
    glBegin(GL_LINE_STRIP);
    glColor3f(0.0, 1.0, 0.0);
    glLineWidth(3.0);

    for(unsigned int i=0; i<nbU; i++){
        Vec p = curve[i];
        glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    }

    glEnd();

    for(unsigned int i=0; i<nbControlPoint; i++){
      TabControlPoint[i]->draw();
    }

    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(unsigned int i=0; i<2; i++){
        Vec p = Vec(TabControlPoint[i]->getX(), TabControlPoint[i]->getY(), TabControlPoint[i]->getZ());
        glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    for(unsigned int i=nbControlPoint-2; i<nbControlPoint; i++){
        Vec p = Vec(TabControlPoint[i]->getX(), TabControlPoint[i]->getY(), TabControlPoint[i]->getZ());
        glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    }
    glEnd();
}
