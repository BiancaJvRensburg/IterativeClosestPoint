#ifndef CURVE_H
#define CURVE_H

#include <QGLViewer/qglviewer.h>
#include "controlpoint.h"

using namespace qglviewer;

class Curve : public QObject
{
    Q_OBJECT

public:
    Curve(unsigned int nbCP, std::vector<Vec>& cntrlPoints);

    void generateBSpline(unsigned int& nbU, unsigned int degree);
    void generateCatmull(unsigned int& nbU);

    Vec* getCurve(){ return curve; }
    Vec& getPoint(unsigned int index){ return curve[index]; }
    unsigned int& getNbU(){ return nbU; }
    ControlPoint* getControlPoint(unsigned int i){ return TabControlPoint[i];}

    void translate(Vec &t);
    void rotate(Quaternion &r);
    void scale(float &s);

    void draw();

public Q_SLOTS:
    void reintialiseCurve();

Q_SIGNALS:
    void curveReinitialised();

private:
    std::vector<ControlPoint*> TabControlPoint;
    unsigned int nbControlPoint;
    Vec *curve;
    unsigned int nbU;

    unsigned int degree;
    std::vector<double> knotVector;
    unsigned int knotIndex;

    // Catmull rom
    void catmullrom();
    void calculateCatmullPoints(Vec& c, double t);

    std::vector<double> generateCatmullKnotVector(double alpha);

    void initConnections();
};

#endif // CURVE_H

