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

    void openOFF(QString filename);

protected:
    virtual void draw();
    virtual void init();

    void updateCamera(const Vec3Df & center, float radius);

    Mesh mesh;
    ManipulatedFrame* viewerFrame;
};
