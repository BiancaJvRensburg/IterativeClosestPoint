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
    void writeJSON(QJsonObject &json);
    void readJSON(const QJsonObject &json);

public Q_SLOTS:
    void openOFF(QString filename, Mesh &m, bool isBase);
    void saveOFF(const QString & filename);
    void registration();
    void registrationSingleStep();
    void rotateX(int);     // rotate 90° around the x axis
    void rotateY(int);
    void rotateZ(int);
    void autoRotate();
    void toUpdate();
    void setBaseAlpha(int);
    void setMeshAlpha(int);

protected:
    virtual void draw();
    virtual void init();

    void initCurve();
    void constructCurve();
    void updateCamera(const Vec3Df & center, float radius);
    ManipulatedFrame* viewerFrame;
    Mesh baseMesh;
    Mesh mesh;
    Curve *curve;
    std::vector<Vec> control;
    int prevX, prevY, prevZ;
    bool isCurve;
};
