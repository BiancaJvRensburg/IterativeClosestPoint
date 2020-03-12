#include "viewer.h"
#include "Triangle.h"
#include "Vec3D.h"
#include <QGLViewer/manipulatedFrame.h>

Viewer::Viewer(QWidget *parent, StandardCamera *cam) : QGLViewer(parent) {
    Camera *c = camera();       // switch the cameras
    setCamera(cam);
    delete c;
    isDrawBase = true;
    isDrawMesh = true;
}

void Viewer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glMultMatrixd(manipulatedFrame()->matrix());

    curve->draw();
    curve->drawControl();

    if(isDrawMesh){
        glColor4f(1.0, 0, 0, 0.5);
        mesh.draw();
    }

    if(isDrawBase){
        glColor4f(1., 1., 1., 0.5);
        baseMesh.draw();
    }

    glPopMatrix();
}

void Viewer::init() {
  restoreStateFromFile();
  setMouseTracking(true);

  viewerFrame = new ManipulatedFrame();
  setManipulatedFrame(viewerFrame);
  setAxisIsDrawn(false);

  glEnable(GL_LIGHTING);
  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  initCurve();
}

void Viewer::openOFF(QString filename, Mesh &m, bool isBase) {
    std::vector<Vec3Df> &vertices = m.getVertices();
    std::vector<Triangle> &triangles = m.getTriangles();

    FileIO::openOFF(filename.toStdString(), vertices, triangles);

    m.init(viewerFrame);
    connect(&m, &Mesh::updateViewer, this, &Viewer::toUpdate);

    // Set the camera
    if(isBase){
        Vec c = m.getBBCentre();
        Vec3Df center = Vec3Df(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
        float radius = m.getBBRadius();
        updateCamera(center, radius);
    }
    else m.alignWithBase(baseMesh);

    std::cout << "File loaded " << std::endl;

    update();
}

void Viewer::saveOFF(const QString & filename){
    FileIO::saveOFF(filename.toStdString(), mesh.getVertices(), mesh.getTriangles());
}

void Viewer::initCurve(){
    std::vector<Vec> control;
     const long nbCP = 9;

    control.push_back(Vec(-5, 5.31, -4.34849));

    control.push_back(Vec(-4.41,1.9687,-2.7316));
    control.push_back(Vec(-3.84341,-0.8484,-1.6));
    control.push_back(Vec(-2.53101,-2.57117,1.67488));

    control.push_back(Vec(-0.125591 , -2.97056 , 2.95159));

    control.push_back(Vec(2.53101,-2.57117,1.67488));
    control.push_back(Vec(3.84341,-0.8484,-1.6));
    control.push_back(Vec(4.41,1.9687,-2.7316));

    control.push_back(Vec(5, 5.31, -4.34849));

    curve = new Curve(nbCP, control);

    unsigned int nbU = 100;
    curve->generateCatmull(nbU);
}

void Viewer::toUpdate(){
    update();
}

void Viewer::updateCamera(const Vec3Df & center, float radius){
    camera()->setSceneCenter(Vec(static_cast<double>(center[0]), static_cast<double>(center[1]), static_cast<double>(center[2])));
    camera()->setSceneRadius(static_cast<double>(radius*1.05f));
    camera()->showEntireScene();
}

void Viewer::registration(){
    mesh.icp(baseMesh);
    update();
}

void Viewer::registrationSingleStep(){
    mesh.icpSingleIteration(baseMesh);
}

void Viewer::rotateX(){
    mesh.rotateAroundAxis(Vec(1,0,0), M_PI/2.0+M_PI);
    update();
}

void Viewer::rotateY(){
    mesh.rotateAroundAxis(Vec(0,1,0), M_PI/2.0+M_PI);
    update();
}

void Viewer::rotateZ(){
    mesh.rotateAroundAxis(Vec(0,0,1), M_PI/2.0+M_PI);
    update();
}

void Viewer::autoRotate(){
    mesh.rotateToBase(baseMesh);
}
