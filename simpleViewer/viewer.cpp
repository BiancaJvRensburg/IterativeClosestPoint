#include "viewer.h"
#include "Triangle.h"
#include "Vec3D.h"
#include <QGLViewer/manipulatedFrame.h>

Viewer::Viewer(QWidget *parent, StandardCamera *cam) : QGLViewer(parent) {
    Camera *c = camera();       // switch the cameras
    setCamera(cam);
    delete c;
}

void Viewer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glMultMatrixd(manipulatedFrame()->matrix());

    glColor3f(1.,1.,1.);
    for(unsigned int i=0; i<meshes.size(); i++) meshes[i]->draw();
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
}

void Viewer::openOFF(QString filename) {
    unsigned long long lastIndex = meshes.size();
    meshes.push_back(new Mesh());
    std::vector<Vec3Df> &vertices = meshes[lastIndex]->getVertices();
    std::vector<Triangle> &triangles = meshes[lastIndex]->getTriangles();
    std::vector< std::vector<unsigned int>> &neighbours = meshes[lastIndex]->getVertexNeighbours();
    std::vector< std::vector<unsigned int>> &vertexTriangles = meshes[lastIndex]->getVertexTriangles();

    FileIO::openOFF(filename.toStdString(), vertices, triangles, neighbours, vertexTriangles);

    meshes[lastIndex]->init();

    // Set the camera
    Vec3Df center;
    double radius;
    MeshTools::computeAveragePosAndRadius(vertices, center, radius);
    updateCamera(center, static_cast<float>(radius));

    update();
}

void Viewer::updateCamera(const Vec3Df & center, float radius){
    camera()->setSceneCenter(Vec(static_cast<double>(center[0]), static_cast<double>(center[1]), static_cast<double>(center[2])));
    camera()->setSceneRadius(static_cast<double>(radius*1.05f));
    camera()->showEntireScene();
}

void Viewer::registration(){
    if(meshes.size()>=2) meshes[0]->icp(meshes[1]);
    update();
}
