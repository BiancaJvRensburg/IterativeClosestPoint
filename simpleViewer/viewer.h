#include <QGLViewer/qglviewer.h>
#include <QGLViewer/manipulatedFrame.h>

#include "meshreader.h"
#include "mesh.h"
#include "standardcamera.h"
#include "curve.h"
#include "controlpoint.h"

using namespace qglviewer;

class Viewer : public QGLViewer
{
    Q_OBJECT

public :
    Viewer(QWidget *parent, StandardCamera *camera);

public Q_SLOTS:
    void openOFF(QString filename);
    void registration();
    void registrationSingleStep();
    void rotateX();     // rotate 90° around the x axis
    void rotateY();
    void rotateZ();
    void autoRotate();
    void toUpdate();
    void increaseNbToDraw();
    void decreaseNbToDraw();

protected:
    virtual void draw();
    virtual void init();

    void initCurve();
    void updateCamera(const Vec3Df & center, float radius);

    std::vector<Mesh*> meshes;
    ManipulatedFrame* viewerFrame;
    unsigned int baseMesh = 0;
    int nbToDraw = 0;
    Curve *curve;
};
