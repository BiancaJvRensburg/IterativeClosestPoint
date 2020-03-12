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
    Mesh& getMesh(bool isBase){ if(isBase) return baseMesh;
                                else return mesh; }

public Q_SLOTS:
    void openOFF(QString filename, Mesh &m, bool isBase);
    void saveOFF(const QString & filename);
    void registration();
    void registrationSingleStep();
    void rotateX();     // rotate 90° around the x axis
    void rotateY();
    void rotateZ();
    void autoRotate();
    void toUpdate();
    void toggleDrawMesh(){ isDrawMesh = !isDrawMesh; update(); }
    void toggleDrawBase(){ isDrawBase = !isDrawBase; update(); }
protected:
    virtual void draw();
    virtual void init();

    void initCurve();
    void updateCamera(const Vec3Df & center, float radius);

    ManipulatedFrame* viewerFrame;
    Mesh baseMesh;
    Mesh mesh;
    Curve *curve;
    bool isDrawMesh;
    bool isDrawBase;
};
