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

    for(unsigned int i=0; i<static_cast<unsigned int>(nbToDraw); i++) meshes[i]->draw();
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

    FileIO::openOFF(filename.toStdString(), vertices, triangles);

    meshes[lastIndex]->init(static_cast<int>(lastIndex), viewerFrame);
    if(lastIndex!=baseMesh) meshes[lastIndex]->alignWithBase(meshes[baseMesh]);
    nbToDraw++;

    connect(meshes[lastIndex], &Mesh::updateViewer, this, &Viewer::toUpdate);

    // Set the camera
    if(meshes.size()==baseMesh+1){
        Vec c = meshes[baseMesh]->getBBCentre();
        Vec3Df center = Vec3Df(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
        float radius = meshes[baseMesh]->getBBRadius();
        updateCamera(center, radius);
    }

    std::cout << "File loaded " << std::endl;

    update();
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
    for(unsigned int i=1; i<meshes.size(); i++) meshes[i]->icp(meshes[baseMesh]);
    update();
}

void Viewer::registrationSingleStep(){
    for(unsigned int i=1; i<meshes.size(); i++) meshes[i]->icpSingleIteration(meshes[baseMesh]);
}

void Viewer::rotateX(){
    meshes[meshes.size()-1]->rotateAroundAxis(Vec(1,0,0), M_PI/2.0+M_PI);
    update();
}

void Viewer::rotateY(){
    meshes[meshes.size()-1]->rotateAroundAxis(Vec(0,1,0), M_PI/2.0+M_PI);
    update();
}

void Viewer::rotateZ(){
    meshes[meshes.size()-1]->rotateAroundAxis(Vec(0,0,1), M_PI/2.0+M_PI);
    update();
}

void Viewer::autoRotate(){
    meshes[meshes.size()-1]->rotateToBase(meshes[baseMesh]);
}

void Viewer::increaseNbToDraw(){
    if(nbToDraw<static_cast<int>(meshes.size())) nbToDraw++;
    update();
}

void Viewer::decreaseNbToDraw(){
    if(nbToDraw>0) nbToDraw--;
    update();
}
