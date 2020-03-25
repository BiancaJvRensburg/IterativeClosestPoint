#ifndef CONTROLPOINT_H
#define CONTROLPOINT_H

#include <QGLViewer/qglviewer.h>
#include <QGLViewer/manipulatedFrame.h>

using namespace qglviewer;

class ControlPoint : public QObject
{

    Q_OBJECT

public:
    ControlPoint();
    ControlPoint(const Vec& p);

    Vec& getPoint(){ return p; }
    const double& getX(){ return p.x; }
    const double& getY(){ return p.y; }
    const double& getZ(){ return p.z; }
    void scale(float& s);
    void translate(Vec &t);
    void move(Vec &t);

    void initialise();
    virtual void draw();

public Q_SLOTS:
    virtual void cntrlMoved();

Q_SIGNALS:
    void cntrlPointTranslated();

protected:
    ManipulatedFrame mf;
    Vec p;
};

#endif // CONTROLPOINT_H
