#include <QGLViewer/qglviewer.h>
#include <QGLViewer/manipulatedFrame.h>

#include "meshreader.h"
#include "mesh.h"
#include "standardcamera.h"

using namespace qglviewer;

class Viewer : public QGLViewer
{
    Q_OBJECT

public :
    Viewer(QWidget *parent, StandardCamera *camera);

public Q_SLOTS:
    void openOFF(QString filename);
    void registration();
    void rotateX();     // rotate 90° around the x axis
    void rotateY();
    void rotateZ();
    void autoRotate();
    void toUpdate();

protected:
    virtual void draw();
    virtual void init();

    void updateCamera(const Vec3Df & center, float radius);

    std::vector<Mesh*> meshes;
    ManipulatedFrame* viewerFrame;
    unsigned int baseMesh = 0;
};
